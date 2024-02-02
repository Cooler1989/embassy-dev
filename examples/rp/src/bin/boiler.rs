use crate::opentherm_interface::Error as OtError;
use crate::opentherm_interface::{
    CHState, FlameState, CommunicationState, DWHState, DataOt, MasterStatus, OpenThermInterface, OpenThermMessageCode, Temperature,
};

const MIN_TEMPERATURE_SETPOINT: Temperature = Temperature::Celsius(16i16);
const BOOST_TEMPERATURE_DEFAULT: Temperature = Temperature::Celsius(24i16);
const SETPOINT_MAX_TEMPERATURE_DEFAULT: Temperature = Temperature::Celsius(80i16);
const BURNER_START_LOW_TRESHOLD: Temperature = Temperature::Celsius(6i16);
const BURNER_STOP_HIGH_TRESHOLD: Temperature = Temperature::Celsius(8i16);

//  The function of boiler control is to maintaing boiler state connected over the OpenTherm
//  interface
//  The most important function is to detect burner start and to react on it with boosting setpoint
//  value for specific period of time to allow to sustain flame - the origin of this requirement is that some
//  boiler producer made a faulty product that is not able to push the heat produced at start of
//  the burner due to high power used at that phase which is around 70-75%
//  After around 15 seconds power can be modulated to lower values and can operate on its own.
pub struct BoilerControl<D: OpenThermInterface> {
    state: BoilerStatus,
    communication_state: CommunicationState,
    device: D,
    boost_setpoint: Temperature,
    setpoint: Temperature,
    max_setpoint: Temperature,
    burner_start_timestamp: u32,
    maintain_ch_state: CHState,
    flame_last_read: Option<FlameState>,
}

enum BoilerStatus {
    Uninitialized, //  to be changed to type level programming
    Idle,
    BurnerStart,
    BurnerModulation,
}

impl<D: OpenThermInterface> BoilerControl<D> {
    pub fn new(opentherm_device: D) -> BoilerControl<D>
    where
        D: OpenThermInterface,
    {
        //  read boiler versions ?
        //  read opentherm versions ?
        //  whichever is mandatory - multiple tries
        //  let state: BoilerStatus::Idle;
        BoilerControl::<D> {
            state: BoilerStatus::Idle,
            communication_state: CommunicationState::StatusExchange,
            device: opentherm_device,
            boost_setpoint: BOOST_TEMPERATURE_DEFAULT,
            max_setpoint: SETPOINT_MAX_TEMPERATURE_DEFAULT,
            setpoint: MIN_TEMPERATURE_SETPOINT,
            burner_start_timestamp: 0u32,
            maintain_ch_state: CHState::Enable(false),
            flame_last_read: None,
        }
    }

    fn set_point_control_internal(&mut self) -> Temperature {
        let temp_to_send = match self.state {
            BoilerStatus::Uninitialized => MIN_TEMPERATURE_SETPOINT,
            BoilerStatus::Idle | BoilerStatus::BurnerModulation => self.setpoint,
            BoilerStatus::BurnerStart => self.setpoint + self.boost_setpoint,
        };
        let temp_to_send = if temp_to_send < SETPOINT_MAX_TEMPERATURE_DEFAULT {
            temp_to_send
        } else {
            SETPOINT_MAX_TEMPERATURE_DEFAULT
        };
        temp_to_send
        //  self.device.write(DataOt::BoilerTemperature(temp_to_send));
    }

    fn state_transition(&mut self) {
        //  Check timer expirations and does maintain state transitions
        //  BoilerStatus::Idle->BurenerStart->BurnerModulation
    }

    pub fn enable_ch(&mut self, enable: CHState) -> Result<(), OtError> {
        self.maintain_ch_state = enable;
        Ok(())
    }
    pub fn set_point(&mut self, temperature: Temperature) -> Result<(), OtError> {
        self.setpoint = temperature;
        Ok(())
    }

    // boiler needs new thread to be maintained and only some calls to change cached state. This is
    // still TODO
    // turn into await / use await driver for reading/writing OpenTherm Bus:
    pub async fn process(&mut self) -> Result<(), OtError> {
        //  let master_status = MasterStatus::new(self.maintain_ch_state, DWHState::Enable(true)); //  temporarily DWH is always on.
        //  let iter_fold = master_status.iter().enumerate().fold(0_u32, |acc, (i, bit_state)| {
        //      let value = acc | ((bit_state as u32) << i);
        //      value
        //  });
        //  log::info!("boiler controller Master status fold: 0x{:x}", iter_fold);
        //  if let Ok(expected_slave_status) = self.device.read(DataOt::MasterStatus(master_status)).await {
        //      //  sent was correct, wait receive now:
        //      let _response = match self.device.listen().await {
        //          Ok(response) => {}
        //          _ => {
        //              log::error!("Boiler::process(): Response to MasterStatus is not valid");
        //          }
        //      };
        //  } else {
        //      log::error!("Boiler failed to report Status!");
        //  }
        //////////////////////////////////////////////////////////////////////////

        self.state_transition(); //  check timer expiration and realize
                                 //  BoilerStatus::BurnerStart->BurnerModulation transition
        let new_state = match self.communication_state {
            CommunicationState::StatusExchange => {
                //  log::info("Boiler p
                let master_status = MasterStatus::new(self.maintain_ch_state, DWHState::Enable(true)); //  temporarily DWH is always on.
                let iter_fold = master_status.iter().enumerate().fold(0_u32, |acc, (i, bit_state)| {
                    let value = acc | ((bit_state as u32) << i);
                    value
                });
                log::info!("boiler controller Master status fold: 0x{:x}", iter_fold);

                log::info!("Boiler Controller sends data: {:?}", master_status);
                if let Ok(expected_slave_status) = self.device.read(DataOt::MasterStatus(master_status)).await {
                    log::info!("Boiler Controller got back data: {:?}", expected_slave_status);
                    match expected_slave_status {
                        DataOt::SlaveStatus(status) => {
                            let flame_read_status = status.get_flame_active();
                            match self.flame_last_read {
                                Some(flame_prev) if flame_prev != flame_read_status => {
                                    todo!();   //  make state transition here
                                },
                                Some(_) => {},
                                None => self.flame_last_read = Some(flame_read_status),
                            }
                        }
                        _ => {
                        }
                    }

                } else {
                    log::error!("Boiler failed to report Status!");
                }
                CommunicationState::Control
            }
            CommunicationState::Control => {
                let temp_to_send = self.set_point_control_internal();
                match self.device.write(DataOt::ControlSetpoint(temp_to_send)).await {
                    Ok(value) => {
                        //  self.device.read(DataOt::Setpoint(Default::default())).await {
                        log::info!("Boiler setpoint sent");
                    }
                    Err(e) => {
                        log::error!("Boiler set setpoint failure");
                    }
                }
                //  self.device.write(DataOt::BoilerTemperature(temp_to_send));
                CommunicationState::Diagnostics
            }
            CommunicationState::Diagnostics => {
                /* if let Ok(value) = */
                match self.device.read(DataOt::BoilerTemperature(Default::default())).await {
                    Ok(value) => {
                        if let DataOt::BoilerTemperature(value) = value {
                            if let Temperature::Celsius(value) = value {
                                log::info!("Boiler temp read: {}", value);
                            }
                        }
                    }
                    Err(e) => {
                        log::error!("Boiler response failure: {:?}", e);
                    }
                }
                CommunicationState::StatusExchange
            }
        };
        self.communication_state = new_state;
        Ok(())
    }
    //  Always or on demand / or combined?
    //  Read back value of setpoint for comparison
    pub fn set_co_temp(&mut self, setpoint: Temperature) {
        self.setpoint = setpoint;
    }
}
