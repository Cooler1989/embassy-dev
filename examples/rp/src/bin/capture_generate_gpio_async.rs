//! This example uses the RP Pico W board Wifi chip (cyw43).
//! Scans Wifi for ssid names.

#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

//  use core::str;

mod boiler;
mod boiler_simulation;
mod manchester;
mod opentherm_interface;

use core::mem;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIN_14, PIN_15, PIN_23, PIN_25, PIO0, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as USBInterruptHandler};
use embassy_time::{Duration, Instant, Timer};
use heapless::Vec;
use manchester::manchester_decode;
use opentherm_interface::{Error as OtError, MessageType, OpenThermInterface, OpenThermMessageCode};
use static_cell::StaticCell;
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

const MANCHESTER_RESOLUTION: Duration = Duration::from_micros(1000u64);
const TOTAL_CAPTURE_OT_FRAME_SIZE: usize = 34usize;
const CAPTURE_OT_FRAME_PAYLOAD_SIZE: usize = 32usize;
const MESSAGE_DATA_VALUE_BIT_LEN: usize = 16usize;
const MESSAGE_DATA_ID_BIT_LEN: usize = 8usize;
const MESSAGE_TYPE_BIT_LEN: usize = 3usize;
const OT_FRAME_SKIP_SPARE: usize = 4usize;

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

trait EdgeCaptureInterface<const N: usize = 128> {
    //  TODO: specify /generically/ idle level or maybe better, return one captured:
    async fn start_capture(
        &mut self,
        timeout_inactive_capture: Duration,
        timeout_till_active_capture: Duration,
    ) -> Result<(InitLevel, Vec<Duration, N>), CaptureError>;
}

struct OpenThermBus<E: EdgeCaptureInterface> {
    edge_capture_drv: E,
}

impl<E: EdgeCaptureInterface> OpenThermBus<E> {
    fn new(edge_capture_driver: E) -> Self {
        Self {
            edge_capture_drv: edge_capture_driver,
        }
    }
}

impl<E: EdgeCaptureInterface> OpenThermInterface for OpenThermBus<E> {
    async fn write(&mut self, cmd: OpenThermMessageCode, data: u32) -> Result<(), OtError> {
        Ok(())
    }
    async fn read(&mut self, cmd: OpenThermMessageCode) -> Result<u32, OtError> {
        match self
            .edge_capture_drv
            .start_capture(10 * MANCHESTER_RESOLUTION, Duration::from_secs(20)) //  a timeout for active capture
            .await
        {
            Ok((init_state, vector)) => {
                log::info!("got data of length: {}", vector.len());
                //  for (i, item) in vector.iter().enumerate() {
                //      log::info!("got vec[{}]: {}", i, item);
                //  }
                match manchester_decode(init_state, vector, MANCHESTER_RESOLUTION) {
                    Ok(vec) => {
                        if vec.len() != TOTAL_CAPTURE_OT_FRAME_SIZE {
                            return Err(OtError::InvalidLength);
                        }
                        //  for (i, item) in vec.iter().enumerate() {
                        //      log::info!("Dec: d[{i}] = {item}");
                        //  }
                        //  Processing starts from LSB / last bit i.e. Stop Bit
                        if vec[0] != true {
                            //  Check Stop Bit
                            log::error!("Decoding error: Stop condition is 'false'. Must be 'true'");
                            return Err(OtError::InvalidStart);
                        }

                        let iter = vec.iter().skip(1);
                        //  expecting start and stop bit already in the vector
                        let folded = iter.clone().take(CAPTURE_OT_FRAME_PAYLOAD_SIZE).enumerate().fold(
                            0u32,
                            |acc, (i, &bit_state)| {
                                let value = acc | ((bit_state as u32) << i);
                                //  log::info!("Acc: 0x{:x}, b:{bit_state}, i: {i}", acc);
                                value
                            },
                        );
                        log::info!("Folded OR: 0x{:x}", folded);
                        //  Check parity for whole OT Frame
                        if folded.count_ones() % 2 == 1 {
                            log::error!("OT Frame Parity Error: 0x{:x}", folded);
                            return Err(OtError::ParityError);
                        }

                        let data_value = iter.clone().take(MESSAGE_DATA_VALUE_BIT_LEN).enumerate().fold(
                            0u16,
                            |acc, (i, &bit_state)| {
                                let value = acc | ((bit_state as u16) << i);
                                //  log::info!("DataId decoding v[{i}] = {} => 0b{:b}", bit_state, value);
                                value
                            },
                        );
                        log::info!("DataValue = 0x{:x}", data_value);

                        let iter = iter.skip(MESSAGE_DATA_VALUE_BIT_LEN);
                        let data_id = iter.clone().take(MESSAGE_DATA_ID_BIT_LEN).enumerate().fold(
                            0_u8,
                            |acc, (i, &bit_state)| {
                                let value = acc | ((bit_state as u8) << i);
                                value
                            },
                        );
                        log::info!("DataId = 0x{:x}", data_id);

                        let data_id: OpenThermMessageCode = match data_id.try_into() {
                            Ok(msg) => msg,
                            Err(_) => {
                                log::error!("Unable to decode Data-Id: 0b{:b}", data_id);
                                return Err(OtError::DecodingError);
                            }
                        };
                        let iter = iter.skip(MESSAGE_DATA_ID_BIT_LEN + OT_FRAME_SKIP_SPARE);

                        let msg_type = iter.clone().enumerate().take(MESSAGE_TYPE_BIT_LEN).clone().fold(
                            0u8,
                            |acc, (i, bit_state)| {
                                let value = acc | (*bit_state as u8) << i;
                                value
                            },
                        );
                        log::info!("MessageType raw value = 0b{:b}", msg_type);

                        let msg_type: MessageType = match msg_type.try_into() {
                            Ok(msg) => msg,
                            Err(_) => {
                                log::error!("Unable to decode MessageType: {msg_type}");
                                return Err(OtError::DecodingError);
                            }
                        };

                        let mut iter = iter.skip(MESSAGE_TYPE_BIT_LEN);
                        let parity = iter.next();
                        let start_bit = iter.next();
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
        Ok(0x00u32)
    }
}

struct RpEdgeCapture<'d, InPin: Pin, const N: usize = 128> {
    input_pin: Input<'d, InPin>,
}

impl<'d, InPin: Pin, const N: usize> EdgeCaptureInterface<N> for RpEdgeCapture<'d, InPin, N> {
    //  TODO:
    //  done: input pin,
    //  max time in total
    //  max time in idle state
    //  consider resolution (wakeup to see if good level is still there)
    //  return timestamp of start and end of the capture
    //  return status: FinishState -> / timeout on active capture, full buffer
    //  add wait init state: low/high, how long
    async fn start_capture(
        &mut self,
        //  input_pin: &mut Input<'static, PIN_15>,
        timeout_inactive_capture: Duration,
        timeout_till_active_capture: Duration,
    ) -> Result<(InitLevel, Vec<Duration, N>), CaptureError> {
        let start_timestamp = Instant::now();
        let init_state = match self.input_pin.is_high() {
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
                        self.input_pin.wait_for_high().await;
                        InitLevel::High
                    }
                    InitLevel::High => {
                        self.input_pin.wait_for_low().await;
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

impl<'d, InPin, const N: usize> RpEdgeCapture<'d, InPin, N>
where
    InPin: Pin,
{
    fn new(input_pin: Input<'d, InPin>) -> Self {
        Self { input_pin }
    }
}

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(2048, log::LevelFilter::Info, driver);
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
async fn capture_input_task(async_input: Input<'static, PIN_15>) -> ! {
    let _init_instant = Instant::now();
    log::info!("Start capture device:");
    //  let mut cap_dev = RpEdgeCapture::<'static, PIN_15, 128>::new(async_input);
    let mut capture_device = RpEdgeCapture::<'static, PIN_15, 128>::new(async_input);

    let mut open_therm_bus = OpenThermBus::new(capture_device);

    loop {
        //  async fn read(&mut self, cmd: OpenThermMessageCode) -> Result<u32, OtError> {
        match open_therm_bus.read(OpenThermMessageCode::Status).await {
            Ok(read_value) => log::info!("OpenThermBus.read() = {read_value}"),
            Err(_) => log::error!("Error: OpenThermBus.read()"),
        }
    }

    //  loop {
    //      match capture_device
    //          .start_capture(10 * MANCHESTER_RESOLUTION, Duration::from_secs(20)) //  a timeout for active capture
    //          .await
    //      {
    //          Ok((init_state, vector)) => {
    //              log::info!("got data of length: {}", vector.len());
    //              //  for (i, item) in vector.iter().enumerate() {
    //              //      log::info!("got vec[{}]: {}", i, item);
    //              //  }
    //              match manchester_decode(init_state, vector, MANCHESTER_RESOLUTION) {
    //                  Ok(vec) => {
    //                      for (i, item) in vec.iter().enumerate() {
    //                          log::info!("Decoded: data[{i}] = {item}");
    //                      }
    //                      if vec[0] != true {
    //                          log::error!("Decoding error: vec[0] is 'false'. Must be 'true'");
    //                      }
    //                      //  expecting start and stop bit already in the vector
    //                      let folded = vec.iter().skip(1).take(CAPTURE_OT_FRAME_PAYLOAD_SIZE).enumerate().fold(0u32, |acc, (i, bit_state)| {
    //                          let value = acc | (*bit_state as u32) << i;
    //                          //log::info!("Acc: {acc}, bit_state: {bit_state}, i: {i}");
    //                          value
    //                      });
    //                      log::info!("Folded OR: 0x{:x}", folded);
    //                      //  now convert folded value into opentherm message.
    //                  }
    //                  Err(err) => {
    //                      log::error!("Decoding error: {:#?}", err);
    //                  }
    //              }
    //          }
    //          Err(_) => {
    //              log::info!("got err");
    //          }
    //      }

    //      //  async_input.wait_for_low().await;
    //      //  let instant_low = Instant::now();
    //      //  log::info!("PIN_14 -> LOW at {}", instant_low);
    //      //  //  TODO: put instant low into ring buffer queue
    //      //  async_input.wait_for_high().await;
    //      //  let instant_high = Instant::now();
    //      //  log::info!("PIN_14 -> HIGH at {}", instant_high);
    //      //  TODO: put instant high into ring buffer queue

    //      //  let end = Instant::now();
    //      //  let ms = end.duration_since(start).as_ticks() / 33;
    //      //  log::info!("[low] done in {} ms", ms);
    //  }
}
#[embassy_executor::task]
async fn generate_toggle_task(mut gpio: Output<'static, PIN_14>) -> ! {
    //  const PERIOD_GENERATE_SEC: usize = 1
    let period = MANCHESTER_RESOLUTION;
    let generate_pattern = 0x98255432u32;
    Timer::after_secs(3).await;
    let mut generate_output_data = Vec::<bool, 128usize>::new();
    //  Sample sequence: 0b11011100:
    //  Sample sequence in manchester: 0b10 10 01 10 10 10 01 01:
    //
    //  generate_output_data.push(true).unwrap();
    //  generate_output_data.push(true).unwrap();
    //  generate_output_data.push(false).unwrap();
    generate_output_data.push(true).unwrap(); // start bit
    for i in 0..(8usize * mem::size_of_val(&generate_pattern)) {
        let bool_bit = ((0x1 << i) & generate_pattern) != 0x0;
        generate_output_data.push(bool_bit).unwrap();
    }
    generate_output_data.push(true).unwrap(); //  end bit
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

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
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
