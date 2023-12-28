use crate::api;

use api::*;

use fixed_macro::types::U56F8;
use fixed::traits::ToFixed;
use embassy_time::{Duration, Timer};
use embassy_rp::pio::{
    Common as PioCommon, Config as ConfigPio, Instance as PioInstance, Direction as PioPinDirection, Irq as PioIrq,
    PioPin, ShiftConfig, ShiftDirection, StateMachine,
};

pub struct PioOpenTherm<'a, PioRx:PioInstance, const SM_RX: usize, PioTx:PioInstance, const SM_TX: usize> {
    //  common_rx_: PioCommon<'a, PioRx>,
    sm_rx_: StateMachine<'a, PioRx, SM_RX>,
    _irq_rx_:PioIrq<'a, PioRx, 3>,

    //  common_tx_: PioCommon<'a, PioTx>,
    sm_tx_: StateMachine<'a, PioTx, SM_TX>,
    irq_tx_:PioIrq<'a, PioTx, 3>,

    //  pin_out: PO,
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

    let mut cfg = ConfigPio::default();
    let out_pin = pio.make_pio_pin(pin);
    cfg.use_program(&pio.load_program(&prg.program), &[&out_pin]);
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

    let mut cfg = ConfigPio::default();
    let in_pin = pio.make_pio_pin(pin);
    let out_pin = pio.make_pio_pin(pin_out);
    cfg.use_program(&pio.load_program(&prg.program), &[&out_pin]);
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


impl<'a, PioRx:PioInstance, const SM_RX: usize, PioTx:PioInstance, const SM_TX: usize> PioOpenTherm<'a, PioRx, SM_RX, PioTx, SM_TX> {
    pub fn new(
        common_rx_arg: &mut PioCommon<'a, PioRx>,
        mut sm_rx_arg: StateMachine<'a, PioRx, SM_RX>,
        irq_rx_arg:PioIrq<'a, PioRx, 3>,
        common_tx_arg: &mut PioCommon<'a, PioTx>,
        mut sm_tx_arg: StateMachine<'a, PioTx, SM_TX>,
        irq_tx_arg: PioIrq<'a, PioTx, 3>,
        pin_rx_arg: impl PioPin,
        pin_rx_out_arg: impl PioPin,
        pin_tx_out_arg: impl PioPin,
    ) -> PioOpenTherm<'a, PioRx, SM_RX, PioTx, SM_TX> {

        setup_pio_task_opentherm_tx(common_tx_arg, &mut sm_tx_arg, pin_tx_out_arg);

        //spawner.spawn(pio_task_opentherm_tx(irq3, sm0)).unwrap();

        setup_pio_task_opentherm_rx(common_rx_arg, &mut sm_rx_arg, pin_rx_arg, pin_rx_out_arg);

        sm_tx_arg.set_enable(true);
        sm_rx_arg.set_enable(true);

        PioOpenTherm {
            //  common_rx_: common_rx_arg,
            sm_rx_: sm_rx_arg,
            _irq_rx_: irq_rx_arg,
            //  common_tx_: common_tx_arg,
            sm_tx_: sm_tx_arg,
            irq_tx_: irq_tx_arg,
        }
    }

    //  async fn run(&mut self) -> ()
    //  {
    //      //  async fn pio_task_opentherm_rx(mut sm: StateMachine<'static, PioRx, SM_RX>) {
    //      //  loop {
    //          let rx = self.sm_rx_.rx().wait_pull().await; //  IRQ may be used as error indicator
    //          //  log::info!("Pulled {:#010x} from FIFO", rx);
    //          let parity_result = u32::count_ones(rx);
    //          if parity_result % 2 == 1 {
    //              //  log::info!("Parity error!");
    //              loop {
    //                  Timer::after(Duration::from_secs(1)).await;
    //              }
    //          }
    //      //  }
    //      //  }
    //  }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum OtError {
    FAIL,
    SUCESS,
    GenericError,
    InvalidData,
    UnknownDataId,
}

impl<'a, PioRx:PioInstance, const SM_RX: usize, PioTx:PioInstance, const SM_TX: usize> OpenThermSlave for PioOpenTherm<'a, PioRx, SM_RX, PioTx, SM_TX> {

    async fn wait_reception_run_callback_respond<F, ErrorSpecific>(&mut self, callback: F) -> Result<(), <PioOpenTherm<'a, PioRx, SM_RX, PioTx, SM_TX> as OpenThermBus>::Error>
    where
        F: Fn(Self::Output) -> Result<Self::Output, ErrorSpecific>,
    {
        match self.rx().await {
            Ok(received_data) => {
                log::info!("OpenTherm Slave got data: {:#010x}", received_data);
                match callback(received_data) {
                    Ok(response) => {
                        log::info!("Slave reponds with data: {:#010x}", response);
                        Timer::after(Duration::from_millis(250)).await;  //  Wait minimum 100 ms to
                                                                         //  realise protocol
                                                                         //  requirements
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

impl<'a, PioRx:PioInstance, const SM_RX: usize, PioTx:PioInstance, const SM_TX: usize> OpenThermBus for PioOpenTherm<'a, PioRx, SM_RX, PioTx, SM_TX> {
    type Error = OtError;
    type Output = OpenThermMessage;

    async fn transact(&mut self, data: Self::Output) -> Result<<PioOpenTherm<'a, PioRx, SM_RX, PioTx, SM_TX> as OpenThermBus>::Output, <PioOpenTherm<'a, PioRx, SM_RX, PioTx, SM_TX> as OpenThermBus>::Error>
    {
        Timer::after(Duration::from_secs(2)).await;
        _ = self.tx(data).await;
        Ok(Self::Output::new(32u32))
    }

    async fn tx(&mut self, data: Self::Output) -> Result<(), <PioOpenTherm<'a, PioRx, SM_RX, PioTx, SM_TX> as OpenThermBus>::Error> {
        //  log::info!("Sending over the wire: {:#010x}", data);

        self.sm_tx_.set_enable(true);
        self.sm_tx_.tx().wait_push(data.get_raw_data_value()).await;
        self.irq_tx_.wait().await;
        self.sm_tx_.restart();
        self.sm_tx_.set_enable(false);
        //  log::info!("PioTxIrq3");
        Ok(())
    }

    async fn rx(&mut self) ->
        Result<<PioOpenTherm<'a, PioRx, SM_RX, PioTx, SM_TX> as OpenThermBus>::Output, <PioOpenTherm<'a, PioRx, SM_RX, PioTx, SM_TX> as OpenThermBus>::Error>
    {
        //  log::info!("Wait data RX...");
        let rx = self.sm_rx_.rx().wait_pull().await; //  IRQ may be used as error indicator
        //  log::info!("Pulled {:#010x} from FIFO", rx);
        let parity_result = u32::count_ones(rx);
        if parity_result % 2 == 1 {
            //  log::info!("Parity error! Assert!");
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }

        Ok(Self::Output::new(rx))
    }
}

