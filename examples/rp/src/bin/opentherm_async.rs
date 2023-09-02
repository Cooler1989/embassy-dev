//! This example shows how to communicate asynchronous OpenTherm interface bus
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

use core::fmt;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pull};
use embassy_rp::peripherals::{PIO0 as PIO_TX, PIO1 as PIO_RX, USB};
//  use embassy_hal_common::{into_ref, Peripheral, PeripheralRef};
use embassy_rp::pio::{
    Common as PioCommon, Config as ConfigPio, Instance as PioInstance, Direction as PioPinDirection, InterruptHandler as InterruptHandlerPio, Irq as PioIrq,
    Pio, PioPin, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::relocate::RelocatedProgram;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_time::{with_timeout, Duration, Timer};
use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => InterruptHandlerPio<PIO_TX>;
    PIO1_IRQ_0 => InterruptHandlerPio<PIO_RX>;
});

#[repr(u8)]
pub enum MessageType {
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

const SM0: usize = 0x0;
//  const SM1: usize = 0x1;

const SM_TX: usize = SM0;
const SM_RX: usize = SM0;

#[repr(u8)]
pub enum OpenThermMessageCode {
    Status = 0x00,
    TSet = 0x01,
    BoilerTemperature = 0x25,
}

#[derive(PartialEq)]
pub struct OpenThermMessage {
    data_value: u32,
}

impl fmt::LowerHex for OpenThermMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let val = self.data_value;
        fmt::LowerHex::fmt(&val, f)
    }
}
// #[derive(PartialEq,Debug)]
impl core::fmt::Debug for OpenThermMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let val = self.data_value;
        fmt::LowerHex::fmt(&val, f)
        //  f.debug_struct("OtMsg").
        //      field("data", &self.data_value).finish()
    }
}

pub trait Error: core::fmt::Debug {
    /// Convert error to a generic I2C error kind
    ///
    /// By using this method, I2C errors freely defined by HAL implementations
    /// can be converted to a set of generic I2C errors upon which generic
    /// code can act.
    fn kind(&self) -> () /*ErrorKind*/;
}

/// This just defines the error type, to be used by the other traits.
pub trait ErrorType {
    /// Error type
    type Error: Error;
}

impl<T: ErrorType> ErrorType for &mut T {
    type Error = T::Error;
}

pub trait OpenThermDevice: ErrorType {
    async fn read(&mut self, cmd: OpenThermMessageCode) -> Result<(), Self::Error> {
        //  place here some conversion and frame assembly
        self.transaction(cmd).await
    }
    async fn transaction(&mut self, cmd: OpenThermMessageCode) -> Result<(), Self::Error>;
}

pub trait OpenThermBus {
    type Error;
    type Output;
    async fn transact(&mut self, data: Self::Output) -> Result<Self::Output, Self::Error>;
    async fn tx(&mut self, data: Self::Output) -> Result<(), Self::Error>;
    async fn rx(&mut self) -> Result<Self::Output, Self::Error>;
}

pub trait OpenThermSlave: OpenThermBus {
    async fn wait_reception_run_callback<F, ErrorSpecific>(&mut self, callback: F) -> Result<(), Self::Error>
    where
        F: Fn(Self::Output) -> Result<Self::Output, ErrorSpecific>;
}

struct PioOpenTherm<'a, PIO_RX:PioInstance, const SM_RX: usize, PIO_TX:PioInstance, const SM_TX: usize> {
    //  common_rx_: PioCommon<'a, PIO_RX>,
    sm_rx_: StateMachine<'a, PIO_RX, SM_RX>,
    irq_rx_:PioIrq<'a, PIO_RX, 3>,

    //  common_tx_: PioCommon<'a, PIO_TX>,
    sm_tx_: StateMachine<'a, PIO_TX, SM_TX>,
    irq_tx_:PioIrq<'a, PIO_TX, 3>,

    //  pin_out: PO,
}

impl<'a, PIO_RX:PioInstance, const SM_RX: usize, PIO_TX:PioInstance, const SM_TX: usize> PioOpenTherm<'a, PIO_RX, SM_RX, PIO_TX, SM_TX> {
    pub fn new(
        common_rx_arg: &mut PioCommon<'a, PIO_RX>,
        mut sm_rx_arg: StateMachine<'a, PIO_RX, SM_RX>,
        irq_rx_arg:PioIrq<'a, PIO_RX, 3>,
        common_tx_arg: &mut PioCommon<'a, PIO_TX>,
        mut sm_tx_arg: StateMachine<'a, PIO_TX, SM_TX>,
        irq_tx_arg: PioIrq<'a, PIO_TX, 3>,
        pin_rx_arg: impl PioPin,
        pin_rx_out_arg: impl PioPin,
        pin_tx_out_arg: impl PioPin,
    ) -> PioOpenTherm<'a, PIO_RX, SM_RX, PIO_TX, SM_TX> {

        setup_pio_task_opentherm_tx(common_tx_arg, &mut sm_tx_arg, pin_tx_out_arg);
        //spawner.spawn(pio_task_opentherm_tx(irq3, sm0)).unwrap();
        setup_pio_task_opentherm_rx(common_rx_arg, &mut sm_rx_arg, pin_rx_arg, pin_rx_out_arg);
        //  setup_pio_task_opentherm_rx<'a>(
        //      pio: &mut PioCommon<'a, PIO_RX>,
        //      sm: &mut StateMachine<'a, PIO_RX, SM_RX>,
        //      pin: impl PioPin,
        //      pin_out: impl PioPin,

        sm_tx_arg.set_enable(true);

        PioOpenTherm {
            //  common_rx_: common_rx_arg,
            sm_rx_: sm_rx_arg,
            irq_rx_: irq_rx_arg,
            //  common_tx_: common_tx_arg,
            sm_tx_: sm_tx_arg,
            irq_tx_: irq_tx_arg,
        }
    }

    async fn run(&mut self) -> ()
    {
        //  async fn pio_task_opentherm_rx(mut sm: StateMachine<'static, PIO_RX, SM_RX>) {
            //  loop {
                let rx = self.sm_rx_.rx().wait_pull().await; //  IRQ may be used as error indicator
                log::info!("Pulled {:#010x} from FIFO", rx);
                let parity_result = u32::count_ones(rx);
                if parity_result % 2 == 1 {
                    log::info!("Parity error!");
                    loop {
                        Timer::after(Duration::from_secs(1)).await;
                    }
                }
            //  }
        //  }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
enum OtError {
    FAIL,
    SUCESS,
}

impl<'a, PIO_RX:PioInstance, const SM_RX: usize, PIO_TX:PioInstance, const SM_TX: usize> OpenThermSlave for PioOpenTherm<'a, PIO_RX, SM_RX, PIO_TX, SM_TX> {

    async fn wait_reception_run_callback<F, ErrorSpecific>(&mut self, callback: F) -> Result<(), <PioOpenTherm<'a, PIO_RX, SM_RX, PIO_TX, SM_TX> as OpenThermBus>::Error>
    where
        F: Fn(Self::Output) -> Result<Self::Output, ErrorSpecific>,
    {
        match self.rx().await {
            Ok(received_data) => {
                log::info!("OpenTherm Slave got data: {:#010x}", received_data);
                match callback(received_data) {
                    Ok(response) => {
                        log::info!("Slave reponds with data: {:#010x}", response);
                        match self.tx(response).await {
                            Ok(()) => {
                                () //  TODO: Should be Err but function expects ()
                            }
                            _ => {
                                log::error!("Error to send the response");
                            }
                        }
                    }
                    Err(_error) => {
                        log::error!("Provided callback is not able to figure out the answer");
                        () //  TODO: Should be Err but function expects ()
                    }
                }
            }
            Err(_error) => {
                log::error!("OpenTherm Slave Error");
                () //  TODO: Should be Err but function expects ()
            }
        }
        Ok(())
    }
}

impl<'a, PIO_RX:PioInstance, const SM_RX: usize, PIO_TX:PioInstance, const SM_TX: usize> OpenThermBus for PioOpenTherm<'a, PIO_RX, SM_RX, PIO_TX, SM_TX> {
    type Error = OtError;
    type Output = OpenThermMessage;
    async fn transact(&mut self, data: Self::Output) -> Result<<PioOpenTherm<'a, PIO_RX, SM_RX, PIO_TX, SM_TX> as OpenThermBus>::Output, <PioOpenTherm<'a, PIO_RX, SM_RX, PIO_TX, SM_TX> as OpenThermBus>::Error> {
        Timer::after(Duration::from_secs(2)).await;
        _ = self.tx(data).await;
        Ok(Self::Output { data_value: 32u32 })
    }
    async fn tx(&mut self, data: Self::Output) -> Result<(), <PioOpenTherm<'a, PIO_RX, SM_RX, PIO_TX, SM_TX> as OpenThermBus>::Error> {
        log::info!("Sending over the wire: {:#010x}", data);
        Ok(())
    }
    async fn rx(&mut self) -> Result<<PioOpenTherm<'a, PIO_RX, SM_RX, PIO_TX, SM_TX> as OpenThermBus>::Output, <PioOpenTherm<'a, PIO_RX, SM_RX, PIO_TX, SM_TX> as OpenThermBus>::Error> {
        Ok(Self::Output { data_value: 0xdaa7 })
    }
}

#[embassy_executor::task]
async fn button(pin: AnyPin) {
    let mut button = Input::new(pin, Pull::Up);
    loop {
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

fn setup_pio_task_opentherm_tx<'a, PIO: PioInstance, const SM: usize>(
    pio: &mut PioCommon<'a, PIO>,
    sm: &mut StateMachine<'a, PIO, SM>,
    pin: impl PioPin,
) {
    // Send data serially to pin with one start bit and stop bit, Manhester 31 bits
    // Cycle is 14 PIO instructions
    let prg = pio_proc::pio_asm!(
        r#"
        .side_set 1 opt
        start:
            jmp get_first
        get_bit:
            out x, 1               ; Always shift out one bit from OSR to X, so we can do conditional jump
        output:
            jmp !x do_zero
        do_one:
            nop         side 1 [6] ;
            jmp continue side 0 [3] ;
        do_zero:
            nop         side 0 [6] ;
            nop         side 1 [3] ;
        continue:
            jmp y-- get_bit
        end:
            nop         side 0 [1] ;
            nop         side 1 [6] ;
            irq 3       side 0 [5] ;
        get_first:
            out x, 1
            nop         side 1 [6] ; Low for 6 cycles (5 delay, +1 for nop)
            nop         side 0 [3] ; High for 4 cycles. 'get_bit' takes another 2 cycles
            set y 31 side 0
            jmp output
        "#,
    );

    let relocated = RelocatedProgram::new(&prg.program);
    let mut cfg = ConfigPio::default();
    let out_pin = pio.make_pio_pin(pin);
    cfg.use_program(&pio.load_program(&relocated), &[&out_pin]);
    cfg.set_out_pins(&[]);
    cfg.set_set_pins(&[]);
    cfg.clock_divider = (U56F8!(125_000_000) / 140 / 100).to_fixed();
    //  cfg.fifo_join = FifoJoin::TxOnly;
    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: ShiftDirection::Left,
    };
    sm.set_pin_dirs(PioPinDirection::Out, &[&out_pin]);
    sm.set_config(&cfg);
}

#[embassy_executor::task]
async fn pio_task_opentherm_tx(mut irq: PioIrq<'static, PIO_TX, 3>, mut sm: StateMachine<'static, PIO_TX, SM_TX>) {
    Timer::after(Duration::from_secs(4)).await;
    sm.set_enable(true);
    let v = 0xffffffff;
    loop {
        log::info!("Pio Start TX");
        sm.set_enable(true);
        sm.tx().wait_push(v).await;
        irq.wait().await;
        log::info!("PioIrq3");

        Timer::after(Duration::from_secs(1)).await;
        sm.set_enable(false);
        sm.restart();
        Timer::after(Duration::from_secs(2)).await;
        //  log::info!("Pushed {:032b} to FIFO", v);
    }
}

fn setup_pio_task_opentherm_rx<'a, PIO: PioInstance, const SM: usize>(
    pio: &mut PioCommon<'a, PIO>,
    sm: &mut StateMachine<'a, PIO, SM>,
    pin: impl PioPin,
    pin_out: impl PioPin,
) {
    // Setup sm1

    // Read 0b10101 repeatedly until ISR is full
    let prg = pio_proc::pio_asm!(
        //  THIS is polarity specific. TODO: Provide separate program for reversed polarity
        //  period is 14 instructions long
        //  Manual push to allow to reset the state when something wrong happens
        //  First prototype doesn't allow to unstack when pin is high for long time. Some amount of
        //  level transitions must happen to reset the state.
        //  IRQ may be used as an error indicator
        r#"
        .side_set 1 opt
        .origin 0
        set y 1
        .wrap_target
        reset:
            set x 31
        check_one:
            jmp pin decrement [5] side 1
            jmp reset
        decrement:
            jmp x-- check_one

        start:
            wait 0 pin 0 side 1
            set x 0 side 0

            set x 31 [1]   side 1   ; Wait until all 32 bits are shifted in
            nop [7]     side 0

        read_jmp_condition:
            nop [3]   ; wait around 3/4 cycle to hit middle of first nibble of next bit
            jmp pin start_of_0  side 1 ; If signal is 1 again, it's another 0 bit, otherwise it's a 1
            jmp start_of_1      side 0

        start_of_1:            ; We are 0.25 bits into a 0 - signal is high
            wait 1 pin 0  side 0     ; Wait for the 1->0 transition - at this point we are 0.5 into the bit
            set y 1
            in y, 1 [1] side 1       ; Emit a 1, sleep 3/4 of a bit
            jmp x-- read_jmp_condition
            jmp exit

        start_of_0:            ; We are 0.25 bits into a 1 - signal is 1
            wait 0 pin 0  side 0     ; Wait for the 0->1 transition - at this point we are 0.5 into the bit
            set y 0
            in y, 1 [1]   side 1     ; Emit a 0, sleep 3/4 of a bit
            jmp x-- read_jmp_condition

        exit:       ;  ignore last bits or rather check if it is there for confirmation
                    ;  change for manual push
            push
        .wrap
        "#,
    );

    let relocated = RelocatedProgram::new(&prg.program);
    let mut cfg = ConfigPio::default();
    let in_pin = pio.make_pio_pin(pin);
    let out_pin = pio.make_pio_pin(pin_out);
    cfg.use_program(&pio.load_program(&relocated), &[&out_pin]);
    cfg.set_jmp_pin(&in_pin);
    cfg.set_in_pins(&[&in_pin]);
    //  cfg.set_out_pins(&[&out_pin]);
    cfg.clock_divider = (U56F8!(125_000_000) / 140 / 100).to_fixed();
    cfg.shift_in = ShiftConfig {
        auto_fill: false,
        threshold: 32,
        direction: ShiftDirection::Left,
    };
    sm.set_pin_dirs(PioPinDirection::In, &[&in_pin]);
    sm.set_pin_dirs(PioPinDirection::Out, &[&out_pin]);
    sm.set_config(&cfg);
}

#[embassy_executor::task]
async fn pio_task_opentherm_rx(mut sm: StateMachine<'static, PIO_RX, SM_RX>) {
    sm.set_enable(true);
    loop {
        let rx = sm.rx().wait_pull().await; //  IRQ may be used as error indicator
        log::info!("Pulled {:#010x} from FIFO", rx);
        let parity_result = u32::count_ones(rx);
        if parity_result % 2 == 1 {
            log::info!("Parity error!");
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }
}

mod version {
    include!(concat!(env!("OUT_DIR"), "/version.rs"));
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);
    //  spawner.spawn(blink(p.PIN_25.into())).unwrap();

    spawner.spawn(logger(driver)).unwrap();
    spawner.spawn(button(p.PIN_28.into())).unwrap();

    //  setup_pio_task_opentherm_rx(&mut common, &mut sm0_rx, p.PIN_2, p.PIN_0);
    //  spawner.spawn(pio_task_opentherm_rx(sm0_rx)).unwrap();

    let mut pio_rx = Pio::new(p.PIO0, Irqs);
    let mut pio_tx = Pio::new(p.PIO1, Irqs);

    let mut pio_ot = PioOpenTherm::new(&mut pio_tx.common, pio_tx.sm0, pio_tx.irq3,
        &mut pio_rx.common, pio_rx.sm0, pio_rx.irq3,
        p.PIN_2, p.PIN_0, p.PIN_1);

    Timer::after(Duration::from_secs(5)).await;

    log::info!("Pio wait OpenTherm transaction");

    //   ============

    loop {
        log::info!("Version: {} {}", version::commit_date(), version::short_sha());
        Timer::after(Duration::from_secs(1)).await;

        let slave_boiler_callback = |input| -> Result<OpenThermMessage, OtError> {
            log::info!("Callback: Simulated boiler received: {:#010x}", input);
            Ok(OpenThermMessage { data_value: 0xcafefefe })
        };

        match pio_ot.wait_reception_run_callback(slave_boiler_callback).await {
            Ok(()) => {
                log::info!("Implement Boiler Simulation reception");
            }
            _ => {
                log::info!("Boiler Simulation error");
            }
        }

        let run_ot = pio_ot.transact(OpenThermMessage { data_value: 24u32 });
        match with_timeout(Duration::from_secs(1), run_ot).await {
            Ok(returned) => match returned {
                Ok(ret) => log::info!("Returned: {:#010x}", ret),
                Err(_err) => log::info!("Transaction Error"),
            },
            Err(_error) => (), /* log::info!("Transaction Timeout")*/
        }
        //  log::info!("Pio OpenTherm transaction is Done");
    }
}
