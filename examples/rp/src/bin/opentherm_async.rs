//! This example shows how to communicate asynchronous using i2c with external chips.
//!
//! Example written for the [`MCP23017 16-Bit I2C I/O Expander with Serial Interface`] chip.
//! (https://www.microchip.com/en-us/product/mcp23017)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, Peripheral};
use embassy_futures::join::join;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pull};
use embassy_rp::pio::{Common, Config as ConfigPio, Direction as PioPinDirection, InterruptHandler as InterruptHandlerPio, Irq as PioIrq, Pio, PioPin, ShiftConfig, ShiftDirection, StateMachine};
use embassy_rp::i2c::{self, Config as ConfigI2c, InterruptHandler};
use embassy_rp::peripherals::USB;
use embassy_rp::peripherals::PIO0;
use embassy_rp::peripherals::I2C1;
use embassy_rp::relocate::RelocatedProgram;
use embassy_rp::usb::{Driver};
use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;
use embassy_time::{Duration, Timer, with_timeout};
use embedded_hal_async::i2c::I2c;
use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<I2C1>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => InterruptHandlerPio<PIO0>;
});

fn swap_nibbles(v: u32) -> u32 {
    let v = (v & 0x0f0f_0f0f) << 4 | (v & 0xf0f0_f0f0) >> 4;
    let v = (v & 0x00ff_00ff) << 8 | (v & 0xff00_ff00) >> 8;
    (v & 0x0000_ffff) << 16 | (v & 0xffff_0000) >> 16
}

#[repr(u8)]
pub enum MessageType
{
    //  master to slave messages
    ReadData = 0x0,
    WriteData = 0x1,
    InvalidData = 0x2,
    Reserved = 0x3,
    // slave to master reponses:
    ReadAck = 0x4,
    WriteAck = 0x5,
    DataInvalid = 0x6,
    UnknownDataId = 0x7,
}

#[repr(u8)]
pub enum OpenThermMessageCode
{
    Status = 0x00,
    TSet = 0x01,
    BoilerTemperature = 0x25,
}

struct OpenThermMessage {
    data_value: u16,
}

pub trait Error: core::fmt::Debug {
    /// Convert error to a generic I2C error kind
    ///
    /// By using this method, I2C errors freely defined by HAL implementations
    /// can be converted to a set of generic I2C errors upon which generic
    /// code can act.
    fn kind(&self) -> ()/*ErrorKind*/;
}

/// This just defines the error type, to be used by the other traits.
pub trait ErrorType {
    /// Error type
    type Error: Error;
}

impl<T: ErrorType> ErrorType for &mut T {
    type Error = T::Error;
}

pub trait OpenThermDevice: ErrorType{
    async fn read( &mut self,
                   cmd: OpenThermMessageCode,
                   ) -> Result<(), Self::Error> {
        //  place here some conversion and frame assembly
        self.transaction(cmd)
            .await
    }
    async fn transaction(
        &mut self,
        cmd: OpenThermMessageCode,
    ) -> Result<(), Self::Error>;
}

pub trait OpenThermBus {
    type Error;
    async fn transact(&mut self, data: u32) -> Result<u32,Self::Error>;
}

struct PioOpenTherm
{
}

impl PioOpenTherm {
    pub fn new() -> PioOpenTherm
    {
        PioOpenTherm{}
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
enum OtError{
    FAIL,
    SUCESS,
}

impl OpenThermBus for PioOpenTherm
{
    type Error = OtError;
    async fn transact(&mut self, data: u32) -> Result<u32,Self::Error>
    {
        Timer::after(Duration::from_secs(2)).await;
        Ok(32u32)
    }
}

#[allow(dead_code)]
mod mcp23017 {
    pub const ADDR: u8 = 0x20; // default addr

    macro_rules! mcpregs {
        ($($name:ident : $val:expr),* $(,)?) => {
            $(
                pub const $name: u8 = $val;
            )*

            pub fn regname(reg: u8) -> &'static str {
                match reg {
                    $(
                        $val => stringify!($name),
                    )*
                    _ => panic!("bad reg"),
                }
            }
        }
    }

    // These are correct for IOCON.BANK=0
    mcpregs! {
        IODIRA: 0x00,
        IPOLA: 0x02,
        GPINTENA: 0x04,
        DEFVALA: 0x06,
        INTCONA: 0x08,
        IOCONA: 0x0A,
        GPPUA: 0x0C,
        INTFA: 0x0E,
        INTCAPA: 0x10,
        GPIOA: 0x12,
        OLATA: 0x14,
        IODIRB: 0x01,
        IPOLB: 0x03,
        GPINTENB: 0x05,
        DEFVALB: 0x07,
        INTCONB: 0x09,
        IOCONB: 0x0B,
        GPPUB: 0x0D,
        INTFB: 0x0F,
        INTCAPB: 0x11,
        GPIOB: 0x13,
        OLATB: 0x15,
    }
}

#[embassy_executor::task]
async fn button(pin:AnyPin)
{
    let mut button = Input::new(pin, Pull::Up);
    loop{
        button.wait_for_low().await;
        log::info!("Buttion pressed!");
        button.wait_for_high().await;
        log::info!("Button released!");
    }
}

#[embassy_executor::task]
async fn blink(pin: AnyPin) {

    let mut led = Output::new(pin, Level::Low);
    loop {
        log::info!("led on!");
        led.set_high();
        Timer::after(Duration::from_secs(1)).await;

        log::info!("led off!");
        led.set_low();
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn logger(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

fn setup_pio_task_sm0<'a>(pio: &mut Common<'a, PIO0>, sm: &mut StateMachine<'a, PIO0, 0>, pin: impl PioPin) {
    // Setup sm0

    // Send data serially to pin
    let prg = pio_proc::pio_asm!(
        //  .side_set 1 opt
        //  .wrap_target
        //      set x, 31
        //  do_one:
        //      nop side 1 [3]
        //      jmp bitloop
        //  do_zero:
        //      nop side 0 [4]
        //  bitloop:
        //      jmp x-- end
        //      out y,1
        //      jmp !y do_zero
        //      jmp do_one
        //  end:
        //      irq 3
        //  .wrap
        r#"
        .side_set 1 opt
        set y 0 side 0
        do_1:
            nop         side 0 [5] ; Low for 6 cycles (5 delay, +1 for nop)
            jmp continue side 1 [3] ; High for 4 cycles. 'get_bit' takes another 2 cycles
        do_0:
            nop         side 1 [5] ; Output high for 6 cycles
            nop         side 0 [3] ; Output low for 4 cycles
        continue:
            jmp y-- get_bit
            irq 3       side 0
            jmp do_1
        get_bit:
            out x, 1               ; Always shift out one bit from OSR to X, so we can
            jmp !x do_0            ; branch on it. Autopull refills the OSR when empty.
            jmp do_1

        "#,
    );
        //  r#"
        //  .side_set 1 opt
        //  .origin 0
        //      set pindirs, 1
        //      set x, 31
        //  .wrap_target


        //  do_zero:
        //  do_one:

        //  bitloop:
        //      out y,1 [18]
        //      jmp x-- bitloop
        //      irq 3
        //  .wrap
        //  "#,

//          r#"
//          .origin 0
//          set pindirs, 1
//          set x, 31
//          .wrap_target
//          out pins,1
//          jmp x-- wrap_target [18]
//          .wrap"#,

    let relocated = RelocatedProgram::new(&prg.program);
    let mut cfg = ConfigPio::default();
    let out_pin = pio.make_pio_pin(pin);
    cfg.use_program(&pio.load_program(&relocated), &[&out_pin]);
    cfg.set_out_pins(&[]);
    cfg.set_set_pins(&[]);
    cfg.clock_divider = (U56F8!(125_000_000) / 200 / 200).to_fixed();
    //  cfg.fifo_join = FifoJoin::TxOnly;
    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: 16,
        direction: ShiftDirection::Right,
    };
    sm.set_pin_dirs(PioPinDirection::Out, &[&out_pin]);
    sm.set_config(&cfg);
}

#[embassy_executor::task]
async fn pio_task_sm0(mut irq: PioIrq<'static, PIO0, 3>, mut sm: StateMachine<'static, PIO0, 0>) {

    Timer::after(Duration::from_secs(4)).await;
    sm.set_enable(true);
    let mut v = 0xffffffff;
    loop {
        log::info!("Pio Start TX");
        sm.set_enable(true);
        sm.tx().wait_push(v).await;
        //  v ^= 0xffff;
        //  irq.wait().await;
        //  log::info!("PioIrq3");

        Timer::after(Duration::from_secs(1)).await;
        sm.set_enable(false);
        sm.restart();
        Timer::after(Duration::from_secs(2)).await;
        //  log::info!("Pushed {:032b} to FIFO", v);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);
    let pio = p.PIO0;
    //  spawner.spawn(blink(p.PIN_25.into())).unwrap();
    spawner.spawn(logger(driver)).unwrap();
    spawner.spawn(button(p.PIN_28.into())).unwrap();

    let Pio {
        mut common,
        irq3,
        mut sm0,
        mut sm1,
        mut sm2,
        ..
    } = Pio::new(pio, Irqs);

    setup_pio_task_sm0(&mut common, &mut sm0, p.PIN_0);
    spawner.spawn(pio_task_sm0(irq3, sm0)).unwrap();

/*
    let Pio {
        mut common,
        sm0: mut sm,
        ..
    } = Pio::new(pio, Irqs);

    let prg = pio_proc::pio_asm!(
        ".origin 0",
        "set pindirs,1",
        ".wrap_target",
        "set y,7",
        "loop:",
        "out x,4",
        "in x,4",
        "jmp y--, loop",
        ".wrap",
    );
    let relocated = RelocatedProgram::new(&prg.program);

    let mut cfg = ConfigPio::default();
    // Pin config
    let out_pin = common.make_pio_pin(p.PIN_16);
    cfg.set_out_pins(&[&out_pin]);
    cfg.set_set_pins(&[&out_pin]);

    cfg.use_program(&common.load_program(&relocated), &[]);
    cfg.clock_divider = (U56F8!(125_000_000) / U56F8!(10_000)).to_fixed();
    cfg.shift_in = ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: ShiftDirection::Left,
    };
    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: ShiftDirection::Right,
    };

    sm.set_config(&cfg);
    sm.set_enable(true);

    let mut dma_out_ref = p.DMA_CH0.into_ref();
    let mut dma_in_ref = p.DMA_CH1.into_ref();
    let mut dout = [0x12345678u32; 29];
    for i in 1..dout.len() {
        dout[i] = (dout[i - 1] & 0x0fff_ffff) * 13 + 7;
    }
    let mut din = [0u32; 29];
    loop {
        let (rx, tx) = sm.rx_tx();
        join(
            tx.dma_push(dma_out_ref.reborrow(), &dout),
            rx.dma_pull(dma_in_ref.reborrow(), &mut din),
        )
        .await;
        for i in 0..din.len() {
            assert_eq!(din[i], swap_nibbles(dout[i]));
        }
        log::info!("Swapped {} words", dout.len());
    }
    */

    Timer::after(Duration::from_secs(5)).await;

    let mut pio_ot = PioOpenTherm::new();
    log::info!("Pio wait OpenTherm transaction");


    //  pub async fn with_timeout<F: Future>(timeout: Duration, fut: F) -> Result<F::Output, TimeoutError>;

    //   ============

    let mut counter = 0;
    loop {
        counter += 1;
        log::info!("Tick {}", counter);
        Timer::after(Duration::from_secs(1)).await;

        let run_ot = pio_ot.transact(24u32);
        match with_timeout(Duration::from_secs(1), run_ot).await {
            Ok(returned) => match returned {
                Ok(ret) => log::info!("Returned: {}", ret),
                Err(err) => log::info!("Transaction Error"),
            },
            Err(error) => log::info!("Transaction Timeout"),
        }
        log::info!("Pio OpenTherm transaction is Done");

    }

    //   ============ I2C Async

    let sda = p.PIN_14;
    let scl = p.PIN_15;

    log::info!("set up i2c ");
    let mut i2c = i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, ConfigI2c::default());

    use mcp23017::*;

    log::info!("init mcp23017 config for IxpandO");
    // init - a outputs, b inputs
    i2c.write(ADDR, &[IODIRA, 0x00]).await.unwrap();
    i2c.write(ADDR, &[IODIRB, 0xff]).await.unwrap();
    i2c.write(ADDR, &[GPPUB, 0xff]).await.unwrap(); // pullups

    let mut val = 1;
    loop {
        let mut portb = [0];

        i2c.write_read(mcp23017::ADDR, &[GPIOB], &mut portb).await.unwrap();
        log::info!("portb = {:02x}", portb[0]);
        i2c.write(mcp23017::ADDR, &[GPIOA, val | portb[0]]).await.unwrap();
        val = val.rotate_left(1);

        // get a register dump
        log::info!("getting register dump");
        let mut regs = [0; 22];
        i2c.write_read(ADDR, &[0], &mut regs).await.unwrap();
        // always get the regdump but only display it if portb'0 is set
        if portb[0] & 1 != 0 {
            for (idx, reg) in regs.into_iter().enumerate() {
                log::info!("{} => {:02x}", regname(idx as u8), reg);
            }
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}
