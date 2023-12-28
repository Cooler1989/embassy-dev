//! This example uses the RP Pico W board Wifi chip (cyw43).
//! Scans Wifi for ssid names.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(async_fn_in_trait)]

//  use core::str;

mod manchester;

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
//  use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed};
use embassy_rp::peripherals::{DMA_CH0, PIN_14, PIN_15, PIN_23, PIN_25, PIO0, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as USBInterruptHandler};
use embassy_time::{Duration, Instant, Timer};
use heapless::Vec;
use manchester::manchester_decode;
use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    USBCTRL_IRQ => USBInterruptHandler<USB>;
});

#[derive(Clone, PartialEq, Debug)]
pub enum GenerateError {
    GenericError,
}
pub enum CaptureError {
    GenericError,
    InvalidData,
    NotEnoughSpace,
}

const MANCHESTER_RESOLUTION: Duration = Duration::from_micros(500u64);

trait EdgeCaptureInterface<const N: usize = 128> {
    //  TODO: specify /generically/ idle level or maybe better, return one captured:
    async fn start_capture_interface() -> Result<(), CaptureError>;
}

struct RpEdgeCapture<const N: usize = 128> {}

#[derive(Clone, PartialEq)]
pub enum InitLevel {
    Low,
    High,
}

pub enum FinishState {
    IdleState,
    TimeoutReached,
    CountReached,
}

impl<const N: usize> RpEdgeCapture<N> {
    fn new() -> Self {
        Self {}
    }
    //  TODO:
    //  done: input pin,
    //  max time in total
    //  max time in idle state
    //  consider resolution (wakeup to see if good level is still there)
    //  return timestamp of start and end of the capture
    //  return status: FinishState -> / timeout on active capture, full buffer
    //  add wait init state: low/high, how long
    async fn start_capture(
        &self,
        input_pin: &mut Input<'static, PIN_15>,
        timeout_inactive_capture: Duration,
        timeout_till_active_capture: Duration,
    ) -> Result<(InitLevel, Vec<Duration, N>), CaptureError> {
        let start_timestamp = Instant::now();
        let init_state = match input_pin.is_high() {
            true => InitLevel::High,
            false => InitLevel::Low,
        };

        let mut capture_timestamp = start_timestamp;
        let mut current_level = init_state.clone();
        let mut timestamps = Vec::<Duration, N>::new();

        //  log::info!("Start edge capture loop...");
        while !timestamps.is_full() {
            //  pub async fn with_timeout<F: Future>(timeout: Duration, fut: F) -> Result<F::Output, TimeoutError> {
            let timeout = match timestamps.is_empty() {
                true => timeout_till_active_capture,
                false => timeout_inactive_capture,
            };
            match embassy_time::with_timeout(timeout, async {
                current_level = match current_level {
                    InitLevel::Low => {
                        input_pin.wait_for_high().await;
                        InitLevel::High
                    }
                    InitLevel::High => {
                        input_pin.wait_for_low().await;
                        InitLevel::Low
                    }
                };
            })
            .await
            {
                Ok(_) => {} //  continue until
                Err(_) => {
                    //  log::warn!("Capture timeout!");
                    //  put last timeouted value into the vector
                    timestamps
                        .insert(0, Instant::now().duration_since(capture_timestamp))
                        .unwrap(); //  here handle error
                    break;
                } //  TODO: check if the error was timeout
            }

            let temporary_timestamp = Instant::now();
            let ms = temporary_timestamp.duration_since(capture_timestamp);
            timestamps.insert(0, ms).unwrap(); //  here handle error
            capture_timestamp = temporary_timestamp;
        }

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
async fn capture_input_task(mut async_input: Input<'static, PIN_15>) -> ! {
    let _init_instant = Instant::now();
    log::info!("Start capture device:");
    let capture_device = RpEdgeCapture::<128>::new();
    loop {
        match capture_device
            .start_capture(&mut async_input, 10 * MANCHESTER_RESOLUTION, Duration::from_secs(20)) //  a timeout for active capture
            .await
        {
            Ok((init_state, vector)) => {
                log::info!("got data");
                for (i, item) in vector.iter().enumerate() {
                    log::info!("got vec[{}]: {}", i, item);
                }
                //  const MANCHESTER_PERIOD_US: usize = 500usize;
                match manchester_decode(init_state, vector, MANCHESTER_RESOLUTION) {
                    Ok(vec) => {
                        for (i, item) in vec.iter().enumerate() {
                            log::info!("Decoded: data[{i}] = {item}");
                        }
                    }
                    Err(err) => {
                        log::error!("Decoding error: {:#?}", err);
                    }
                }
            }
            Err(_) => {
                log::info!("got err");
            }
        }

        //  async_input.wait_for_low().await;
        //  let instant_low = Instant::now();
        //  log::info!("PIN_14 -> LOW at {}", instant_low);
        //  //  TODO: put instant low into ring buffer queue
        //  async_input.wait_for_high().await;
        //  let instant_high = Instant::now();
        //  log::info!("PIN_14 -> HIGH at {}", instant_high);
        //  TODO: put instant high into ring buffer queue

        //  let end = Instant::now();
        //  let ms = end.duration_since(start).as_ticks() / 33;
        //  log::info!("[low] done in {} ms", ms);
    }
}
#[embassy_executor::task]
async fn generate_toggle_task(mut gpio: Output<'static, PIN_14>) -> ! {
    //  const PERIOD_GENERATE_SEC: usize = 1
    let period = MANCHESTER_RESOLUTION;
    Timer::after_secs(3).await;
    let mut generate_output_data = Vec::<bool, 128usize>::new();
    //  Sample sequence: 0b11011100:
    //  Sample sequence in manchester: 0b10 10 01 10 10 10 01 01:
    //
    //  generate_output_data.push(true).unwrap();
    //  generate_output_data.push(true).unwrap();
    //  generate_output_data.push(false).unwrap();
    generate_output_data.push(true).unwrap();
    generate_output_data.push(true).unwrap();
    generate_output_data.push(false).unwrap();
    generate_output_data.push(false).unwrap();
    generate_output_data.push(true).unwrap();
    generate_output_data.push(true).unwrap();
    loop {
        log::info!("Start generating output data");
        gpio.set_low(); //  generate idle state:
        Timer::after_secs(3).await;
        log::info!("Generate continue generation at {}", Instant::now());
        generate_manchester_output(&mut gpio, &generate_output_data, period)
            .await
            .unwrap();

        gpio.set_low(); //  generate idle state after

        log::info!("Generate wait start at {}", Instant::now());
        Timer::after_secs(10).await;
        log::info!("...");
    } // | 1 | 1 | 0 | 1 | 1 | 1 | 0 | 0 | 1 |
} //_____|^|_|^|_._  .   |^|_|^|_._|^|_|^|^|_.___
  //_______|   |   |   |   |   |   |   |   |   |
  //  Vec::<bool, 128usize>::new();
  //  fn new(pins: [Output<'a, AnyPin>; 8]) -> Self {

async fn generate_manchester_output(
    gpio: &mut Output<'static, PIN_14>,
    data: &Vec<bool, 128usize>,
    period: Duration,
) -> Result<(), GenerateError> {
    for item in data {
        if *item == true {
            gpio.set_high();
        } else {
            gpio.set_low();
        }
        Timer::after(period).await;
        gpio.toggle();
        Timer::after(period).await;
    }
    Ok(())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    Timer::after_secs(2).await;
    log::info!("Init at {}", Instant::now());

    let mut gpio_output = Output::new(p.PIN_14, Level::Low);
    gpio_output.set_low(); //  Initial idle state for open therm bus
    let async_input = Input::new(p.PIN_15, Pull::Up);

    unwrap!(spawner.spawn(capture_input_task(async_input)));
    unwrap!(spawner.spawn(generate_toggle_task(gpio_output)));

    log::info!("Both capture and generate started");

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
    //  generate_manchester_output(gpio_output, &generate_output_data).unwrap();

    const VEC_SIZE_MANCHESTER: usize = 128usize;
    let mut wire_state_periods = Vec::<Duration, VEC_SIZE_MANCHESTER>::new();
    let now = Instant::now();
    wire_state_periods.push(Instant::now().duration_since(now)).unwrap();
    wire_state_periods.push(Instant::now().duration_since(now)).unwrap();
    wire_state_periods.push(Instant::now().duration_since(now)).unwrap();

    let _output = manchester_decode(InitLevel::Low, wire_state_periods, MANCHESTER_RESOLUTION);

    log::info!("Infinite loop sstart");
    loop {
        Timer::after_secs(1).await;
    }
    loop {
        //  async_input.wait_for_low().await;
        control.gpio_set(0, true).await;
        let instant_low = Instant::now();
        log::info!("PIN_14 -> LOW at {}", instant_low);
        Timer::after_secs(1).await;
        //  async_input.wait_for_high().await;
        control.gpio_set(0, false).await;
        let instant_high = Instant::now();
        log::info!("PIN_14 -> HIGH at {}", instant_high);
        Timer::after_secs(1).await;
    }
}
