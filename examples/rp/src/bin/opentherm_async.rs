//! This example shows how to communicate asynchronous OpenTherm interface bus
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pull};
use embassy_rp::peripherals::{PIO0 as PioTx, PIO1 as PioRx, USB};
//  use embassy_hal_common::{into_ref, Peripheral, PeripheralRef};
use embassy_rp::pio::{ InterruptHandler as InterruptHandlerPio, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};
use embassy_opentherm::{OpenThermMessage, OpenThermSlave};   //  TODO: rename it to opentherm-async
use embassy_opentherm::pio_opentherm::{PioOpenTherm, OtError};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => InterruptHandlerPio<PioTx>;
    PIO1_IRQ_0 => InterruptHandlerPio<PioRx>;
});

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

//  #[embassy_executor::task]
//  async fn pio_task_opentherm_tx(mut irq: PioIrq<'static, PioTx, 3>, mut sm: StateMachine<'static, PioTx, SM_TX>) {
//      Timer::after(Duration::from_secs(4)).await;
//      sm.set_enable(true);
//      let v = 0xffffffff;
//      loop {
//          log::info!("Pio Start TX");
//          sm.set_enable(true);
//          sm.tx().wait_push(v).await;
//          irq.wait().await;
//          log::info!("PioIrq3");
//
//          Timer::after(Duration::from_secs(1)).await;
//          sm.set_enable(false);
//          sm.restart();
//          Timer::after(Duration::from_secs(2)).await;
//          //  log::info!("Pushed {:032b} to FIFO", v);
//      }
//  }

//  #[embassy_executor::task]
//  async fn pio_task_opentherm_rx(mut sm: StateMachine<'static, PioRx, SM_RX>) {
//      sm.set_enable(true);
//      loop {
//          let rx = sm.rx().wait_pull().await; //  IRQ may be used as error indicator
//          log::info!("Pulled {:#010x} from FIFO", rx);
//          let parity_result = u32::count_ones(rx);
//          if parity_result % 2 == 1 {
//              log::info!("Parity error!");
//              loop {
//                  Timer::after(Duration::from_secs(1)).await;
//              }
//          }
//      }
//  }

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

    //  log::info!("Version: {} {}", version::commit_date(), version::short_sha());

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

        let slave_boiler_callback = |input| -> Result<OpenThermMessage, OtError> {
            log::info!("Callback: Simulated boiler received: {:#010x}", input);
            Ok(OpenThermMessage::new( 0xcafefefe ))
        };

        match pio_ot.wait_reception_run_callback_respond(slave_boiler_callback).await {
            Ok(()) => {
                log::info!("Implement Boiler Simulation reception");
            }
            _ => {
                log::info!("Boiler Simulation error");
            }
        }
    }
}
