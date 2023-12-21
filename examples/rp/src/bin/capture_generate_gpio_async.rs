//! This example uses the RP Pico W board Wifi chip (cyw43).
//! Scans Wifi for ssid names.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(async_fn_in_trait)]

//  use core::str;

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_time::{Timer, Instant};
use embassy_net::Stack;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output, Input, Pull};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as USBInterruptHandler};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_14, PIN_15, PIN_25, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use heapless::Vec;
use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    USBCTRL_IRQ => USBInterruptHandler<USB>;
});

pub enum CaptureError {
    GenericError,
    InvalidData,
    NotEnoughSpace,
}

trait EdgeCaptureInterface<const N:usize = 128>
{
    //  TODO: specify /generically/ idle level or maybe better, return one captured:
    async fn start_capture() -> Result<(), CaptureError>;
}

struct RpEdgeCapture< const N:usize = 128> {
}

#[derive(Clone)]
pub enum InitLevel {
    Low,
    High
}

pub enum FinishState {
    IdleState,
    TimeoutReached,
    CountReached
}

impl<const N:usize> RpEdgeCapture<N> {
    fn new() -> Self
    {
        Self{}
    }
    //  TODO:
    //  input pin,
    //  max time in total
    //  max time in idle state
    //  consider resolution (wakeup to see if good level is still there)
    //  return timestamp of start and end of the capture
    //  return status: FinishState ->
    //  add wait init state: low/high, how long
    async fn start_capture(self, mut input_pin: Input<'static, PIN_15>) -> Result<(InitLevel, Vec<usize,N>), CaptureError>
    {
        let start_timestamp = Instant::now();
        let init_state = match input_pin.is_high() {
            true => {
                InitLevel::High
            },
            false => {
                InitLevel::Low
            },
        };

        let mut capture_timestamp = start_timestamp;
        let mut current_level = init_state.clone();
        let mut count: usize = 0;
        let mut timestamps = Vec::<usize,N>::new();

        while count < N {
            current_level = match current_level {
                InitLevel::Low => {
                    input_pin.wait_for_high().await;
                    InitLevel::High
                },
                InitLevel::High => {
                    input_pin.wait_for_low().await;
                    InitLevel::Low
                }
            };
            let temporary_timestamp = Instant::now();
            let ms = temporary_timestamp.duration_since(capture_timestamp).as_ticks();
            timestamps.push(ms as usize).unwrap();  //  here return error
            capture_timestamp = temporary_timestamp;
            count+=1;
        };

        timestamps.push(234).unwrap();
        timestamps.push(345).unwrap();
        Ok((init_state, timestamps))
    }
}

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static, PIN_23>, PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
async fn capture_input_task(mut input: Input<'static, PIN_15>) -> ! {
    let _init_instant = Instant::now();
    loop {
        input.wait_for_low().await;
        let instant_low = Instant::now();
        log::info!("PIN_14 -> LOW at {}", instant_low);
        //  TODO: put instant low into ring buffer queue
        input.wait_for_high().await;
        let instant_high = Instant::now();
        log::info!("PIN_14 -> HIGH at {}", instant_high);
        //  TODO: put instant high into ring buffer queue

        //  let end = Instant::now();
        //  let ms = end.duration_since(start).as_ticks() / 33;
        //  log::info!("[low] done in {} ms", ms);
    }
}
#[embassy_executor::task]
async fn generate_toggle_task(mut gpio: Output<'static, PIN_14>) -> ! {
    loop {
        gpio.set_high();
        Timer::after_secs(1).await;
        gpio.set_low();
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {

    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();
    log::info!("Init at {}", Instant::now());

    let mut led = Output::new(p.PIN_14, Level::Low);
    let mut async_input = Input::new(p.PIN_15, Pull::Up);

    log::info!("wait_for_high. Turn on LED");
    led.set_high();
    unwrap!(spawner.spawn(generate_toggle_task(led)));
    //  unwrap!(spawner.spawn(capture_input_task(async_input)));

    let fw = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    let state = make_static!(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    //  let mut scanner = control.scan().await;
    //  while let Some(bss) = scanner.next().await {
    //      if let Ok(ssid_str) = str::from_utf8(&bss.ssid) {
    //          log::info!("scanned {}", ssid_str);
    //      }
    //  }

    let capture_device = RpEdgeCapture::<128>::new();
    capture_device.start_capture(async_input);

    loop {
        //  async_input.wait_for_low().await;
        control.gpio_set(0, true).await;
        let instant_low = Instant::now();
        log::info!("PIN_14 -> LOW at {}", instant_low);
        //  async_input.wait_for_high().await;
        control.gpio_set(0, false).await;
        let instant_high = Instant::now();
        log::info!("PIN_14 -> HIGH at {}", instant_high);
    }
}

//  TODO: Write some tests
//
//  test decoder of the duration table into manchaster sync protocol (check in WIO project)
//
