use crate::opentherm_interface::Error as OtError;
use crate::opentherm_interface::{
    CHState, FlameState, CommunicationState, DWHState, DataOt, MasterStatus, OpenThermInterface, OpenThermMessageCode, Temperature,
};
use embassy_time::{Duration, Instant, Timer};

//  TODO:
//  a) implement verification if the setpoint last requested to be set is the same as the one last
//  read, if not try to fix it or report

const MIN_TEMPERATURE_SETPOINT: Temperature = Temperature::Celsius(16i16);
const TEMP_COOL_DOWN_DECREASE: Temperature = Temperature::Celsius(10i16);
const BOOST_TEMPERATURE_DEFAULT: Temperature = Temperature::Celsius(24i16);
const SETPOINT_MAX_TEMPERATURE_DEFAULT: Temperature = Temperature::Celsius(80i16);
const BURNER_START_LOW_TRESHOLD: Temperature = Temperature::Celsius(6i16);
const BURNER_STOP_HIGH_TRESHOLD: Temperature = Temperature::Celsius(8i16);
const BURNER_BOOST_SETPOINT_DURTAION: Duration = Duration::from_secs(30);

//  The function of boiler control is to maintaing boiler state connected over the OpenTherm
//  interface
//  The most important function is to detect burner start and to react on it with boosting setpoint
//  value for specific period of time to allow to sustain flame - the origin of this requirement is that some
//  boiler producer made a faulty product that is not able to push the heat produced at start of
//  the burner due to high power used at that phase which is around 70-75%
//  After around 15 seconds power can be modulated to lower values and can operate on its own.
pub struct BoilerControl<D: OpenThermInterface> {
    state: BoilerState,
    communication_state: CommunicationState,
    device: D,
    boost_setpoint: Temperature,
    setpoint: Temperature,
    max_setpoint: Temperature,
    maintain_ch_state: CHState,
    flame_last_read: Option<FlameState>,
    flame_start_timestamp: Option<Instant>,  //  TODO: Set to None if flame goes of, calculate and
                                             //  store duration
    flame_stop_timestamp: Option<Instant>,
}

#[derive(Debug)]
enum BoilerError {
    InvalidState,
    MissingFlameStatus,
}

enum InputEvents {
    FlameOn,
    FlameSame,
    FlameOff,
    ChOn,
    ChOff,
    DwhOn,
    DwhOff,
}

#[derive(Debug, Copy, Clone, PartialEq)]
enum BoilerState {
    Uninitialized, //  to be changed to type level programming
    StandBy,  //  where the CH = 0
    Idle,   //  where the CH = 1
    BurnerStartupBoost,
    BurnerModulation,
    BurnerOffCooldown,
    DwhActive,  //  flame active, CH = 0, DWH = 1,
    ErrorStateTrap,
}

impl<D: OpenThermInterface> BoilerControl<D> {
    pub fn new(opentherm_device: D) -> BoilerControl<D>
    where
        D: OpenThermInterface,
    {
        //  read boiler versions ?
        //  read opentherm versions ?
        //  whichever is mandatory - multiple tries
        //  let state: BoilerState::Idle;
        BoilerControl::<D> {
            state: BoilerState::Idle,
            communication_state: CommunicationState::StatusExchange,
            device: opentherm_device,
            boost_setpoint: BOOST_TEMPERATURE_DEFAULT,
            max_setpoint: SETPOINT_MAX_TEMPERATURE_DEFAULT,
            setpoint: MIN_TEMPERATURE_SETPOINT,
            maintain_ch_state: CHState::Enable(false),
            flame_last_read: None,
            flame_start_timestamp: None,
            flame_stop_timestamp: None,
        }
    }

    fn set_point_control_internal(&self) -> Temperature {
        let temp_to_send = match self.state {
            BoilerState::Uninitialized | BoilerState::StandBy | BoilerState::ErrorStateTrap => MIN_TEMPERATURE_SETPOINT,
            BoilerState::Idle | BoilerState::BurnerModulation | BoilerState::DwhActive => self.setpoint,
            BoilerState::BurnerOffCooldown => self.setpoint - TEMP_COOL_DOWN_DECREASE,
            BoilerState::BurnerStartupBoost => self.setpoint + self.boost_setpoint,
        };
        let temp_to_send = if temp_to_send < SETPOINT_MAX_TEMPERATURE_DEFAULT {
            temp_to_send
        } else {
            SETPOINT_MAX_TEMPERATURE_DEFAULT
        };
        temp_to_send
        //  self.device.write(DataOt::BoilerTemperature(temp_to_send));
    }

    //  Idle => BurnerStartupBoost
    //  TODO:
    //    a)return the information about the need to execute setpoint
    //    b) Consider splitting transition to state and handling the transition actions for cleaner
    //      approach
    fn state_transition(&mut self, slave_status : Option<DataOt>) -> Result<(), BoilerError> {
        //  Check timer expirations and does maintain state transitions
        //  BoilerState::Idle->BurenerStart->BurnerModulation
        let prev_state = self.state;
        self.state = match self.state {
            BoilerState::Uninitialized => {  //  -> BurnerStartupBoost / DwhActive
                BoilerState::Idle
            }
            BoilerState::Idle => {  //  -> BurnerStartupBoost / DwhActive
                //  a) flame goes up => BurnerStartupBoost or DwhActive
                //  b) CH = 1 => BurnerStartupBoost
                //  c) DWH = 1 => DwhActive
                if let Some(DataOt::SlaveStatus(status)) = slave_status {
                    if status.get_flame_active() == FlameState::Active(true)  {
                        if status.get_ch_active() == CHState::Enable(true) {
                            BoilerState::BurnerStartupBoost
                        } else if status.get_dwh_active() == DWHState::Enable(true) {
                            BoilerState::DwhActive
                        }
                        else {
                            BoilerState::ErrorStateTrap
                        }
                    }
                    else { // when flame is off do not change the state:
                        self.state
                    }
                }
                else { // When status is not given, preserve the state
                       // TODO: maybe consider checking incorrect input argument
                    self.state
                }
            },
            BoilerState::BurnerStartupBoost => {  //  -> BurnerModulation / BurnerCooldown / DwhActive
                //  a) timeout 30s => BurnerModulation
                //  b) flame goes down => BurnerCooldown
                //  c) flipping a switch ch -> dwh in status => DwhActive
                match self.flame_start_timestamp {  //  Check if this should be turned off due to
                                                    //  timeout
                    Some(burner_start) if Instant::now().duration_since(burner_start) > BURNER_BOOST_SETPOINT_DURTAION => {
                        todo!();   // check flame status and ch active bit here
                        BoilerState::BurnerModulation
                    }
                    Some(burner_start) => { self.state },//  leave it as it was if the duration is
                    //  shorter
                    _ => {
                        todo!();  //  Handle the critical state here. If the flame was detected it
                                  //  the timesetamp vatiable should have Some(value)
                                  //  The possible action is to do one cycle to process
                                  //  ErrorStatTrape
                                  //  with safety signals and after that turn off the boiler
                        return Err(BoilerError::InvalidState);
                    }
                }
            },
            BoilerState::BurnerModulation => {  //  -> BurnerCooldown / DwhActive / Idle
                //  a) flame goes down => BurnerCooldown
                //  b) flipping a switch ch -> dwh in status => DwhActive
                if let Some(DataOt::SlaveStatus(status)) = slave_status {
                    if status.get_flame_active() == FlameState::Active(true)  {
                        if status.get_ch_active() == CHState::Enable(true) {
                            self.state  //  preserve
                        } else if status.get_dwh_active() == DWHState::Enable(true) {
                            BoilerState::DwhActive
                        }
                        else {
                            BoilerState::ErrorStateTrap
                        }
                    }
                    else { // when flame is off go to cool down state
                        self.flame_stop_timestamp = Some(Instant::now());
                        BoilerState::BurnerOffCooldown
                    }
                }
                else { // When status is not given, preserve the state
                       // TODO: maybe consider checking incorrect input argument
                    self.state
                }
            },
            BoilerState::BurnerOffCooldown => {  //  -> Idle / DwhActive
                if let Some(DataOt::SlaveStatus(status)) = slave_status {
                    if status.get_flame_active() == FlameState::Active(true)  {
                        if status.get_dwh_active() == DWHState::Enable(true) {
                            BoilerState::DwhActive
                        }
                        else {
                            todo!();  // not expected non-DWH
                            self.state
                        }
                    }
                    else {
                        self.state
                    }
                }
                else {
                    self.state
                }
            },
            BoilerState::StandBy => {  //  -> Idle(CH=1) / DwhActive(dwh=1)  TODO: check dhw state
                //  Standby is then the CH = 0, DWH = 0 and flame is OFF
                todo!();
                self.state
            },
            BoilerState::DwhActive => {
                //  fn handle_dwh_active_state_transition(slave_status: DataOt)
                //  handle_dwh_active_state_transition(slave_status)
                let new_self_state = if let Some(DataOt::SlaveStatus(status)) = slave_status {
                    let intermediate_self_state = if status.get_flame_active() == FlameState::Active(true)  {
                        if status.get_dwh_active() == DWHState::Enable(false) {
                            //  handle situation:
                            todo!();
                            self.state
                        }
                        else {
                            todo!();
                            self.state
                        }
                    }
                    else {
                        BoilerState::Idle
                    };
                    intermediate_self_state
                }
                else {
                    self.state
                };
                new_self_state
            }
            BoilerState::ErrorStateTrap => {
                self.state
            },
            //  s => s,
        };
        if prev_state != self.state {
            log::info!("Boiler State Transition: {:?}->{:?}", prev_state, self.state);
        }

        todo!();  // redundant conditions. Does it make sense?
        if let Some(DataOt::SlaveStatus(status)) = slave_status {
            let flame_read_status = status.get_flame_active();
            match self.flame_last_read {
                Some(flame_prev) if flame_prev != flame_read_status => {
                    self.state = BoilerState::BurnerStartupBoost;
                    self.flame_last_read = Some(flame_read_status);
                    self.flame_start_timestamp = Some(Instant::now());
                },
                Some(_) => {},
                None => self.flame_last_read = Some(flame_read_status),
            }
        }
        //  The state change may also come from external condition of a statte change in
        //  the boiler itself
        self.state = match self.flame_last_read {
            Some(flame) if flame == FlameState::Active(false) => {
                //  Flame was detected to go off:
                BoilerState::BurnerOffCooldown
            },
            Some(_) => {
                self.state   //  leave the same state as before
            },
            None => {
                log::error!("Error: boiler should have flame status at this point");
                return Err(BoilerError::MissingFlameStatus);
            }
        };
        Ok(())
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

        self.state_transition(None); //  check timer expiration and realize
                                 //  BoilerState::BurnerStartupBoost->BurnerModulation transition
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
                    self.state_transition(Some(expected_slave_status)); //  check timer expiration and realize

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
