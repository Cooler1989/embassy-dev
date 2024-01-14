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

use boiler::BoilerControl;
use boiler_simulation::BoilerSimulation;
use core::mem;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIN_12, PIN_13, PIN_14, PIN_15, PIN_23, PIN_25, PIO0, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as USBInterruptHandler};
use embassy_time::{Duration, Instant, Timer};
use heapless::Vec;
use manchester::manchester_decode;
use opentherm_interface::{CHState, Temperature};
use opentherm_interface::{
    DataOt, Error as OtError, MessageType, OpenThermInterface, OpenThermMessage, OpenThermMessageCode,
};
use opentherm_interface::{
    CAPTURE_OT_FRAME_PAYLOAD_SIZE, MESSAGE_DATA_ID_BIT_LEN, MESSAGE_DATA_VALUE_BIT_LEN, MESSAGE_TYPE_BIT_LEN,
    OT_FRAME_SKIP_SPARE,
};
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

enum TriggerError {
    GenericError,
}

pub enum CaptureError {
    GenericError,
    InvalidData,
    NotEnoughSpace,
}

const MANCHESTER_RESOLUTION: Duration = Duration::from_millis(10u64);

const TOTAL_CAPTURE_OT_FRAME_SIZE: usize = 34usize;

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

trait EdgeTriggerInterface {
    async fn trigger(&mut self, iterator: impl Iterator<Item = bool>) -> Result<(), TriggerError>;
}

trait EdgeCaptureInterface<const N: usize = 128> {
    //  TODO: specify /generically/ idle level or maybe better, return one captured:
    async fn start_capture(
        &mut self,
        timeout_inactive_capture: Duration,
        timeout_till_active_capture: Duration,
    ) -> Result<(InitLevel, Vec<Duration, N>), CaptureError>;
}

struct OpenThermBus<E: EdgeCaptureInterface, T: EdgeTriggerInterface> {
    edge_capture_drv: E,
    edge_trigger_drv: T,
    idle_level: u8,
}

impl<E: EdgeCaptureInterface, T: EdgeTriggerInterface> OpenThermBus<E, T> {
    //  TODO: introduce idle level parameter
    fn new(edge_capture_driver: E, edge_trigger_driver: T) -> Self {
        Self {
            edge_capture_drv: edge_capture_driver,
            edge_trigger_drv: edge_trigger_driver,
            idle_level: 0x0_u8,
        }
    }

    async fn transact(&mut self, message_type: MessageType, cmd: DataOt) -> Result<DataOt, OtError> {
        let msg = OpenThermMessage::new_from_data_ot(message_type, cmd);
        let result = match msg {
            Ok(msg) => {
                //  let mut count = 0u32;
                //  for item in msg.iter() {
                //      count += 1u32;
                //      //  log::info!("msg[{count}] = {}", item as bool);
                //  }

                let manchester_adapter = ManchesterIteratorAdapter::new(msg.iter());
                match self.edge_trigger_drv.trigger(manchester_adapter).await {
                    Ok(()) => {
                        //  Successful send:
                        //  read and return value or error as it is
                        self.listen().await //  await response
                    }
                    Err(error) => {
                        log::error!("Trigger error!");
                        return Err(OtError::BusError);
                    }
                }
            }
            _ => {
                return Err(OtError::DecodingError);
            }
        };
        result
    }
}

impl<E: EdgeCaptureInterface, T: EdgeTriggerInterface> OpenThermInterface for OpenThermBus<E, T> {
    async fn listen(&mut self) -> Result<DataOt, OtError> {
        match self
            .edge_capture_drv
            .start_capture(10 * MANCHESTER_RESOLUTION, Duration::from_secs(20)) //  a timeout for active capture
            .await
        {
            Ok((init_state, vector)) => {
                //  caputure
                log::info!("got data of length: {}", vector.len());

                //  let received_message = OpenThermMessage::new_from_iter(ManchesterIteratorAdapter::new(vector.iter()));
                let received_message = match manchester_decode(init_state, vector, MANCHESTER_RESOLUTION) {
                    Ok(vec) => {
                        if vec.len() != TOTAL_CAPTURE_OT_FRAME_SIZE {
                            return Err(OtError::InvalidLength);
                        }
                        //  Check Stop Bit
                        if vec[0] != true {
                            log::error!("Decoding error: Stop condition is 'false'. Must be 'true'");
                            return Err(OtError::InvalidStart);
                        }

                        let opentherm_frame_iterator = vec.iter().skip(1).take(CAPTURE_OT_FRAME_PAYLOAD_SIZE);
                        let ret_msg = OpenThermMessage::new_from_iter(opentherm_frame_iterator);

                        let iter = vec
                            .iter()
                            .skip(1 + MESSAGE_DATA_VALUE_BIT_LEN + MESSAGE_DATA_ID_BIT_LEN + OT_FRAME_SKIP_SPARE);
                        let mut iter = iter.skip(MESSAGE_TYPE_BIT_LEN);
                        let parity = iter.next();
                        let start_bit = iter.next();
                        match start_bit {
                            Some(level) => {
                                if *level == false {
                                    return Err(OtError::DecodingError);
                                }
                            }
                            _ => {
                                return Err(OtError::DecodingError);
                            }
                        }
                        //  Return positive value:
                        ret_msg
                    }
                    Err(err) => {
                        log::error!("Decoding error: {:#?}", err);
                        return Err(OtError::DecodingError);
                    }
                };
            }
            Err(_) => {
                log::info!("got err");
            }
        }
        log::error!("Unexpected result capture error");
        return Err(OtError::UnexpectedResult);
    }

    async fn write(&mut self, cmd: DataOt) -> Result<(), OtError> {
        let _ = self.transact(MessageType::WriteData, cmd).await?;
        Ok(())
    }
    async fn read(&mut self, cmd: DataOt) -> Result<DataOt, OtError> {
        self.transact(MessageType::ReadData, cmd).await
    }
}

struct ManchesterIteratorAdapter<I: Iterator<Item = bool>> {
    iterator: I,
    next_bit: Option<bool>,
}

impl<I: Iterator<Item = bool>> ManchesterIteratorAdapter<I> {
    fn new(iterator_arg: I) -> Self {
        Self {
            iterator: iterator_arg,
            next_bit: None,
        }
    }
}

impl<I: Iterator<Item = bool>> Iterator for ManchesterIteratorAdapter<I> {
    type Item = bool;
    fn next(&mut self) -> Option<Self::Item> {
        let ret = match self.next_bit {
            None => match self.iterator.next() {
                None => None, //  depleted
                Some(bit) => {
                    self.next_bit = Some(bit);
                    Some(!bit)
                }
            },
            Some(bit) => {
                self.next_bit = None;
                Some(bit)
            }
        };
        ret
    }
}

struct RpEdgeTrigger<'d, OutPin: Pin> {
    output_pin: Output<'d, OutPin>,
}

//  fn resolve_folded(iterator: impl Iterator<Item = bool> + Clone) -> u32 {
//      let mut count = 0u32;
//      for item in iterator.clone() {
//          count += 1u32;
//      }
//      log::info!("Edge Trigger count: {count}");
//      count
//  }

impl<'d, OutPin: Pin> EdgeTriggerInterface for RpEdgeTrigger<'d, OutPin> {
    async fn trigger(&mut self, iterator: impl Iterator<Item = bool>) -> Result<(), TriggerError> {
        let period = MANCHESTER_RESOLUTION;
        self.output_pin.set_low(); //  generate idle state:
        Timer::after(period).await; //  await one period in idle state

        //  This loop already handles the manchester encoding:
        let mut count = 0u32;
        for item in iterator {
            // iterator.for_each( async move |item| {
            if item == true {
                self.output_pin.set_high();
            } else {
                self.output_pin.set_low();
            }
            Timer::after(period).await;
            count += 1u32;
        }
        log::info!("Edge Trigger sent count: {count}");

        self.output_pin.set_low(); //  generate idle state after
        Timer::after(period).await; //  await one period in idle state
        Ok(())
    }
}

impl<'d, OutPin> RpEdgeTrigger<'d, OutPin>
where
    OutPin: Pin,
{
    fn new(output_pin: Output<'d, OutPin>) -> Self {
        Self { output_pin }
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
async fn boiler_simulation_task(async_input: Input<'static, PIN_12>, async_output: Output<'static, PIN_13>) -> ! {
    let _init_instant = Instant::now();
    log::info!("Start capture device:");
    let mut capture_device = RpEdgeCapture::new(async_input);
    let mut trigger_device = RpEdgeTrigger::new(async_output);

    let mut open_therm_bus = OpenThermBus::new(capture_device, trigger_device);
    let mut boiler_simulation = BoilerSimulation::new();

    loop {
        let response = match open_therm_bus.listen().await {
            Ok(read_value) => {
                let cmd = boiler_simulation.process(read_value).unwrap();
                Some(cmd)
            }
            Err(_) => {
                log::error!("Error: OpenThermBus.read()");
                None
            }
        };

        //  Send response:
        Timer::after_secs(3).await;

        match response {
            Some(cmd) => {
                open_therm_bus.write(cmd).await.unwrap();
            }
            None => {
                ();
            }
        }
    }
}

#[embassy_executor::task]
async fn boiler_controller_task(async_input: Input<'static, PIN_14>, mut async_output: Output<'static, PIN_15>) -> ! {
    //  const PERIOD_GENERATE_SEC: usize = 1
    let mut capture_device = RpEdgeCapture::new(async_input);
    let mut trigger_device = RpEdgeTrigger::new(async_output);

    let mut open_therm_bus = OpenThermBus::new(capture_device, trigger_device);
    let mut boiler_controller = BoilerControl::new(open_therm_bus);

    boiler_controller.set_point(Temperature::Celsius(16));
    boiler_controller.enable_ch(CHState::Enable(true));
    loop {
        log::info!("Process Boiler controller call");
        boiler_controller.process().await;
        Timer::after_secs(10).await;

        //  open_therm_bus
        //      .write(OpenThermMessageCode::Status, 0x00_u32)
        //      .await
        //      .unwrap();
        //  Timer::after_millis(50).await;
        //  match open_therm_bus.read(OpenThermMessageCode::Status).await {
        //      Ok(read_value) => log::info!("OpenThermBus.read() = {read_value}"),
        //      Err(_) => log::error!("Error: OpenThermBus.read()"),
        //  }
    }
    //  let period = MANCHESTER_RESOLUTION;
    //  let generate_pattern = 0x10255432_u32;

    //  let mut generate_output_data = Vec::<bool, 128usize>::new();
    //  generate_output_data.push(true).unwrap(); // start bit
    //  for i in 0..(8usize * mem::size_of_val(&generate_pattern)) {
    //      let bool_bit = ((0x1 << i) & generate_pattern) != 0x0;
    //      generate_output_data.push(bool_bit).unwrap();
    //  }
    //  generate_output_data.push(true).unwrap(); //  stop bit
    //  loop {
    //      log::info!("Start generating output data");
    //      async_output.set_low(); //  generate idle state:
    //      Timer::after_secs(3).await;
    //      log::info!("Generate continue generation at {}", Instant::now());
    //      generate_manchester_output(&mut async_output, &generate_output_data, period)
    //          .await
    //          .unwrap();

    //      async_output.set_low(); //  generate idle state after

    //      log::info!("Generate wait start at {}", Instant::now());
    //      Timer::after_secs(10).await;
    //      log::info!("...");
    //  } // | 1 | 1 | 0 | 1 | 1 | 1 | 0 | 0 | 1 |
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

    let mut gpio_output = Output::new(p.PIN_15, Level::Low);
    let gpio_input = Input::new(p.PIN_14, Pull::Up);
    gpio_output.set_low(); //  Initial idle state for open therm bus
    let async_input = Input::new(p.PIN_12, Pull::Up);
    let async_output = Output::new(p.PIN_13, Level::Low);

    unwrap!(spawner.spawn(boiler_simulation_task(async_input, async_output)));
    unwrap!(spawner.spawn(boiler_controller_task(gpio_input, gpio_output)));

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

    const VEC_SIZE_MANCHESTER: usize = 128usize;
    let mut wire_state_periods = Vec::<Duration, VEC_SIZE_MANCHESTER>::new();
    let now = Instant::now();
    wire_state_periods.push(Instant::now().duration_since(now)).unwrap();
    wire_state_periods.push(Instant::now().duration_since(now)).unwrap();
    wire_state_periods.push(Instant::now().duration_since(now)).unwrap();

    let _output = manchester_decode(InitLevel::Low, wire_state_periods, MANCHESTER_RESOLUTION);

    log::info!("Infinite emppty loop sstart");
    loop {
        Timer::after_secs(1).await;
    }
}
