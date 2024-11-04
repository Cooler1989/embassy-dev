#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::i2c::InterruptHandler as I2cInterruptHandler;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::Timer;
use embassy_rp::gpio;
use gpio::{Level, Output};
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config, Handler};
use rand::Rng;
use static_cell::StaticCell;
use usbd_hid::descriptor::{MouseReport, SerializedDescriptor};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    I2C1_IRQ => I2cInterruptHandler<embassy_rp::peripherals::I2C1>;
});

const MIDDLE_POSITION : u8 = 0;

enum MotorRoller458 {
    First = 0x64,
}
impl Into<u16> for MotorRoller458 {
    fn into(self) -> u16 {
        0x64
    }
}

struct ScrollDiff(i8);

static CHANNEL: StaticCell<Channel<NoopRawMutex, ScrollDiff, 1>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);
    let channel = CHANNEL.init(Channel::new());

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("HID keyboard example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    // You can also add a Microsoft OS descriptor.
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut request_handler = MyRequestHandler {};
    let mut device_handler = MyDeviceHandler::new();

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    // Create classes on the builder.
    let config = embassy_usb::class::hid::Config {
        report_descriptor: MouseReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut state, config);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let (reader, mut writer) = hid.split();

    // Do stuff with the class!
    let in_fut = async {
        let mut rng = RoscRng;

        loop {
            //  receive
            match channel.receiver().receive().await {
                ScrollDiff(scroll_diff) => {
                    let report = MouseReport {
                        buttons: 0,
                        //  x: rng.gen_range(-40..40), // random small x movement
                        x: 0,
                        y: 0,
                        //  y: rng.gen_range(-40..40), // random small y movement
                        //  wheel: rng.gen_range(-10..10), // random small scroll movement
                        wheel: scroll_diff,
                        pan: 0,
                    };
                    // Send the report.
                    match writer.write_serialize(&report).await {
                        Ok(()) => {}
                        Err(e) => warn!("Failed to send report: {:?}", e),
                    }
                }
            };
            _ = Timer::after_millis(100).await;
        }
    };

    let out_fut = async {
        reader.run(false, &mut request_handler).await;
    };

    let mut led = Output::new(p.PIN_25, Level::Low);
    let led_toggler = async {
        loop {
            //  info!("wait_for_high. Turn on LED");
            led.toggle();
            _ = Timer::after_secs(1).await;
        }
    };

    // Setup i2c driver:
    let sda = p.PIN_14;
    let scl = p.PIN_15;

    let config = embassy_rp::i2c::Config::default();
    let mut bus = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, config);

    let motor_control_loop = async {

        //  Read buffer
        let write_buffer: [u8;1] = [0x01];
        let mut read_buffer: [u8;1] = [0; 1];
        let result = bus
            .write_read_async(MotorRoller458::First, write_buffer, &mut read_buffer)
            .await;
        match result {
            Ok(()) => {
                //  log::info!("Read results: {}, {}", read_buffer[0], read_buffer[1]);
            },
            Err(_err) => {
                loop{};
            }
        }
        _ = Timer::after_secs(1).await;

        //  Set position:
        let write_buffer: [u8;5] = [0x80, 0x00, MIDDLE_POSITION/*position*/, 0x00, 0x00];  //  select register
        let result = bus
            .write_async(MotorRoller458::First, write_buffer)
            .await;
        match result {
            Ok(()) => {
                log::info!("Read results Ok");
            },
            Err(_err) => {
                loop{};
            }
        }

        //  Set motor mode to position:
        let write_buffer: [u8;2] = [0x01, 0x02];
        let result = bus
            .write_async(MotorRoller458::First, write_buffer)
            .await;
        match result {
            Ok(()) => {
                //  log::info!("Write results Ok");
            },
            Err(_err) => {
                loop{};
            }
        }

        _ = Timer::after_secs(1).await;
        //  Motor ON:
        let write_buffer: [u8;2] = [0x00, 0x01];
        let result = bus
            .write_async(MotorRoller458::First, write_buffer)
            .await;
        match result {
            Ok(()) => {
                //  log::info!("Write results Ok");
            },
            Err(_err) => {
                loop{};
            }
        }
        _ = Timer::after_secs(1).await;

        loop {
            //  Read position:
            let write_buffer: [u8;1] = [0x90];  //  Register value
            let mut read_buffer: [u8;4] = [0; 4];
            let result = bus
                .write_read_async(MotorRoller458::First, write_buffer, &mut read_buffer)
                .await;
            match result {
                Ok(()) => {
                    log::info!("Read results: {}, {}", read_buffer[0], read_buffer[1]);
                    let position = (read_buffer[2] as i16) - (MIDDLE_POSITION as i16);
                    //  Get motor position,
                    //  Send diff data:
                    channel.sender().send(ScrollDiff(position as i8)).await;
                },
                Err(_err) => {
                    loop{};
                }
            }
            _ = Timer::after_millis(100).await;
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut,
        join(in_fut,
            join(out_fut,
                join(led_toggler,
                    motor_control_loop)))).await;
}

struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            info!("Device configured, it may now draw up to the configured current limit from Vbus.")
        } else {
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}
