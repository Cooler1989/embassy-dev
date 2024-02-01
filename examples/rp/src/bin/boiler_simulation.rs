use crate::opentherm_interface::Error as OtError;
use crate::opentherm_interface::{
    CHState, CommunicationState, DataOt, MessageType, OpenThermInterface, OpenThermMessage, OpenThermMessageCode,
    Temperature,
};
use crate::opentherm_interface::{DWHState, FlameState, SlaveStatus};
use embassy_time::{Duration, Instant, Timer};

const TEMP_CHANGE_VALUE: Temperature = Temperature::Celsius(1);
const TEMP_CHANGE_RATE: Duration = Duration::from_secs(10);
const BOILER_MAX_TEMP: Temperature = Temperature::Celsius(60);
const TEMP_UPPER_TURN_OFF_HIST: Temperature = Temperature::Celsius(8);
const TEMP_LOWER_TURN_ON_HIST: Temperature = Temperature::Celsius(6);

//  #[derive(Default)]
pub struct BoilerSimulation {
    //  struct for testing operation of the driver used for boiler control
    boiler_temperature: Temperature,
    setpoint: Temperature,
    on_off_state: CHState,
    dwh_active: DWHState,
    flame: FlameState,
    last_process_call: Instant,
}

impl BoilerSimulation {
    pub fn new() -> Self {
        Self {
            boiler_temperature: Temperature::Celsius(18),
            setpoint: Temperature::Celsius(16),
            on_off_state: CHState::Enable(false),
            dwh_active: DWHState::Enable(true),
            flame: FlameState::Active(false),
            last_process_call: Instant::now(),
        }
    }

    fn maintain_state(&mut self) {
        let period_last_call = Instant::now().duration_since(self.last_process_call);
        //  only update state after at least a second interval
        if period_last_call > TEMP_CHANGE_RATE {
            self.last_process_call = Instant::now();
            // elevate integer calculation 1000 times
            if self.flame == FlameState::Active(true) {
                self.boiler_temperature = self.boiler_temperature + TEMP_CHANGE_VALUE;
            } else {
                self.boiler_temperature = self.boiler_temperature - TEMP_CHANGE_VALUE;
            }
        }

        // update the boiler state
        self.flame = match self.boiler_temperature {
            Temperature::Celsius(t)
                if self.flame == FlameState::Active(true)
                    && Temperature::Celsius(t) >= self.setpoint + TEMP_UPPER_TURN_OFF_HIST =>
            {
                FlameState::Active(false)
            }
            Temperature::Celsius(t)
                if self.flame == FlameState::Active(false)
                    && Temperature::Celsius(t) <= self.setpoint - TEMP_LOWER_TURN_ON_HIST =>
            {
                FlameState::Active(true)
            }
            Temperature::Celsius(_) => self.flame,
        };
    }

    //  accepts OpenTherm message
    //  returns the one that would be sent in response by the Boiler
    pub fn process(&mut self, msg: OpenThermMessage) -> Result<OpenThermMessage, OtError> {
        let cmd = msg.get_data();
        self.maintain_state();

        let correct_type_response = match msg.get_type() {
            MessageType::ReadData => MessageType::ReadAck,
            MessageType::WriteData => MessageType::WriteAck,
            _ => MessageType::InvalidData,
        };

        //  Command handling:
        let data = match cmd {
            DataOt::MasterStatus(rx_status) => {
                let slave_status = SlaveStatus::new(self.on_off_state, self.dwh_active, self.flame);
                if rx_status.ch_enable == CHState::Enable(true) {
                    log::info!("Boiler Simulation: Got CH Enable signal set true ------------->  CH enable");
                }
                self.on_off_state = rx_status.ch_enable;
                self.dwh_active = rx_status.dwh_enable;
                if msg.get_type() != MessageType::ReadData {
                    log::error!("Boiler Simulation: Invalid msg type for MasterStatus");
                    return Err(OtError::IncompatibleTypeData);
                }
                let iter_fold = slave_status.iter().enumerate().fold(0_u32, |acc, (i, bit_state)| {
                    let value = acc | ((bit_state as u32) << i);
                    value
                });
                log::info!("boiler simulation Slave status fold response: 0x{:x}", iter_fold);
                DataOt::SlaveStatus(slave_status)
            }
            DataOt::ControlSetpoint(setpoint) => {
                let response = DataOt::ControlSetpoint(self.setpoint);
                self.setpoint = setpoint;
                if msg.get_type() != MessageType::WriteData {
                    log::error!("Boiler Simulation: Invalid msg type for Setpoint");
                    return Err(OtError::IncompatibleTypeData);
                }
                response
            }
            DataOt::BoilerTemperature(temperature) => {
                let response = DataOt::BoilerTemperature(self.boiler_temperature);
                if msg.get_type() != MessageType::ReadData {
                    log::error!("Boiler Simulation: Invalid msg type for BoilerTemperature");
                    todo!();
                    return Err(OtError::IncompatibleTypeData);
                }
                response
            }
            _ => {
                log::error!("Boiler simulation: command not implemented");
                return Err(OtError::CommandNotSupported);
            }
        };
        OpenThermMessage::new_from_data_ot(correct_type_response, data)
    }
}
