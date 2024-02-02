//! This example uses the RP Pico W board Wifi chip (cyw43).
//! Connects to specified Wifi network and creates a TCP endpoint on port 1234.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(async_fn_in_trait)]

use core::str::from_utf8;

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::Ipv4Address;
use embassy_net::{Config, Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use static_cell::make_static;
use embassy_rp::clocks::RoscRng;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::reason_codes::ReasonCode,
    //  utils::rng_generator::CountingRng,
};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const WIFI_NETWORK: &str = "EmbassyTest";
const WIFI_PASSWORD: &str = "V8YxhKt5CdIAJFud";

const CLIENT_ID: &'static str = "client_rp_id";
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    let p = embassy_rp::init(Default::default());

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
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = Config::dhcpv4(Default::default());
    //let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
    //    address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 69, 2), 24),
    //    dns_servers: Vec::new(),
    //    gateway: Some(Ipv4Address::new(192, 168, 69, 1)),
    //});

    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    // Init network stack
    let stack = &*make_static!(Stack::new(
        net_device,
        config,
        make_static!(StackResources::<2>::new()),
        seed
    ));

    unwrap!(spawner.spawn(net_task(stack)));

    loop {
        //control.join_open(WIFI_NETWORK).await;
        match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

    // Wait for DHCP, not necessary when using static IP
    info!("waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    info!("DHCP is now up!");

    // And now we can use it!

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];


    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        control.gpio_set(0, false).await;
        Timer::after(Duration::from_millis(100)).await;

        info!("Connecting to TCP:1883...");
        let remote_endpoint = (Ipv4Address::new(192, 168, 7, 1), 1883);
        let r = socket.connect(remote_endpoint).await;
        if let Err(e) = r {
            control.gpio_set(0, true).await;
            info!("connect error: {:?}", e);
            Timer::after_secs(1).await;
            continue;
        }

        let rng = RoscRng;
        let mut config = ClientConfig::<5, RoscRng>::new(rust_mqtt::client::client_config::MqttVersion::MQTTv5, rng);
        config.add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
        config.add_client_id(CLIENT_ID);
        config.max_packet_size = 100;
        let mut recv_buffer = [0; 128];
        let mut write_buffer = [0; 128];
        let mut client = MqttClient::<_, 5, _>::new(socket, &mut write_buffer, 80, &mut recv_buffer, 80, config);

        //  control.gpio_set(0, true).await;

        let delay = Duration::from_secs(1);
        let short_delay = Duration::from_millis(100);
        loop {
            Timer::after(delay).await;
            let temperature_string: String<32> = String::try_from("21").unwrap();
            match client
                .send_message(
                    "temperature/1",
                    temperature_string.as_bytes(),
                    rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1,
                    true,
                )
                .await
            {
                Ok(()) => {}
                Err(mqtt_error) => match mqtt_error {
                    ReasonCode::NetworkError => {
                        //  log::info!("MQTT Network Error");
                        control.gpio_set(0, true).await;
                        Timer::after(short_delay).await;
                        control.gpio_set(0, false).await;
                        Timer::after(short_delay).await;
                        control.gpio_set(0, true).await;
                        Timer::after(short_delay).await;
                        control.gpio_set(0, false).await;
                        Timer::after(short_delay).await;
                        continue;
                    }
                    _ => {
                        //  log::info!("Other MQTT Error: {:?}", mqtt_error);
                        control.gpio_set(0, true).await;
                        Timer::after(short_delay).await;
                        control.gpio_set(0, false).await;
                        Timer::after(short_delay).await;
                        control.gpio_set(0, true).await;
                        Timer::after(short_delay).await;
                        control.gpio_set(0, false).await;
                        Timer::after(short_delay).await;
                        control.gpio_set(0, true).await;
                        Timer::after(short_delay).await;
                        control.gpio_set(0, false).await;
                        Timer::after(short_delay).await;
                        continue;
                    }
                },
            }
            control.gpio_set(0, true).await;
            Timer::after(delay).await;

            control.gpio_set(0, false).await;
            Timer::after(delay).await;

            //  let n = match socket.read(&mut buf).await {
            //      Ok(0) => {
            //          warn!("read EOF");
            //          break;
            //      }
            //      Ok(n) => n,
            //      Err(e) => {
            //          warn!("read error: {:?}", e);
            //          break;
            //      }
            //  };

            //  info!("rxd {}", from_utf8(&buf[..n]).unwrap());

            //  match socket.write_all(&buf[..n]).await {
            //      Ok(()) => {}
            //      Err(e) => {
            //          warn!("write error: {:?}", e);
            //          break;
            //      }
            //  };

        }
    }
}
