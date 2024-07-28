//! This example shows how to use SPI (Serial Peripheral Interface) in the RP2040 chip.
//! No specific hardware is specified in this example. If you connect pin 11 and 12 you should get the same data back.

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::spi::{Config, Spi};
use embassy_time::Timer;
use core::convert::Infallible;
use embassy_rp::gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};
use cortex_m_semihosting::hprintln;

use pn532::{requests::SAMMode, spi::SPIInterface, Pn532, Request};

use embedded_hal::digital::v2::OutputPin as OutputPin_v2;

pub struct NoOpCS<'a>{
    gpio: Output<'a>,
}

impl<'a> NoOpCS<'a> {
    pub fn new(gpio: Output<'a>) -> Self {
        Self{gpio}
    }
}

impl<'a> OutputPin_v2 for NoOpCS<'a> {
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.gpio.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.gpio.set_high();
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    hprintln!("Hello World!");

    let miso = p.PIN_12;
    let mosi = p.PIN_11;
    let clk = p.PIN_14;
    let cs = Output::new(p.PIN_9, Level::High);
    //  let cs = NoOpCS::new(Output::new(p.PIN_7, Level::Low));

    let mut spi = Spi::new(p.SPI1, clk, mosi, miso, p.DMA_CH2, p.DMA_CH3, Config::default());
    spi.set_frequency(200_000);
    let interface = SPIInterface {
        spi,
        cs,
    };
    //  hprintln!("Creat new Pn532");
    //  let mut pn532: Pn532<_, (), 32> = Pn532::new_async(interface);
    //  hprintln!("Pn532: process_async");
    //  if let Err(e) = pn532.process_async(&Request::GET_FIRMWARE_VERSION, 4).await {
    //      hprintln!("Could not read firmware version PN532");
    //  }
    //  hprintln!("Read firmware version PN532");

    //  if let Err(e) = pn532.process_async(&Request::sam_configuration(SAMMode::Normal, false), 0).await {
    //      // info!("Could not initialize PN532: {e:?}")
    //      hprintln!("Could not initialize PN532");
    //  }
    //  hprintln!("Phn532: sam_configuration done");

    //  loop {
    //      let tx_buf = [1_u8, 2, 3, 4, 5, 6];
    //      let mut rx_buf = [0_u8; 6];
    //      spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
    //      info!("{:?}", rx_buf);
    //      Timer::after_secs(1).await;
    //  }

    let mut led = Output::new(p.PIN_22, Level::Low);
    loop {
        hprintln!("led on!");
        led.set_high();
        Timer::after_secs(1).await;

        hprintln!("led off!");
        led.set_low();
        Timer::after_secs(1).await;
    }
}
