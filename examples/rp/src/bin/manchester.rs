use embassy_time::Duration;
use heapless::Vec;

use super::InitLevel;

#[derive(Debug)]
pub enum CodingError {
    InitialLevelError,
    PeriodEncodingError,
    FinishExpectedError,
    NoSpaceAvailable,
    WrongInput,
}

//  __|'' manchester 'one'
//  ''|__ manchester 'zero'
//
//    __|''''|__ manchester '10'
//    ''|____|'' manchester '01'
//
//    Assumptions: first edge is always the transition from idle state and it marks start of the
//    bit.

//  TODO: Prepare data first - remove first and last long periods
pub fn manchester_decode<const N: usize /*Vec max size*/>(
    level: InitLevel,
    mut wire_state_periods: Vec<Duration, N>,
    resolution: Duration, /*period duration which specifies smalles amount of time the line can state in one state*/
) -> Result<Vec<bool, N>, CodingError> {
    let mut output = Vec::<bool, N>::new();
    //  discard first cycle as it is not relevant because
    //  capture should start before first relevant level transition
    //  TODO: possibly do it upstream
    if wire_state_periods.is_empty() {
        return Err(CodingError::WrongInput);
    }
    let _ignore = wire_state_periods.pop().unwrap();
    //  Return if initial level is not idle level
    if level != InitLevel::Low {
        return Err(CodingError::InitialLevelError);
    }

    //  Initial level is very important as it determines all other transitions
    //  TODO: make it more redundant and safe to mistake one level transition and still have most
    //  of the data correct
    let mut level = level; // init level later used as mut to handle iterations

    //  First half of initial 'one' bit:
    match wire_state_periods.pop() {
        Some(pop_period) => {
            if (100 * pop_period < 95 * resolution) || (100 * pop_period > 105 * resolution) {
                return Err(CodingError::PeriodEncodingError);
            }
        }
        _ => {
            return Err(CodingError::PeriodEncodingError);
        }
    }
    output.push(true).unwrap(); //  First 'one' bit

    //  Loop over the periods; each loop starts in the middle of already determined bit and based
    //  on the next period detects next bit level. If needed it pops next periods until it lands
    //  in the middle of just determined bit.
    while let Some(pop_period) = wire_state_periods.pop() {
        //  Check if the period is of value around of the resolution:
        if (100 * pop_period > 95 * resolution) && (100 * pop_period < 105 * resolution) {
            let output_candidate = match level {
                InitLevel::Low => {
                    true //  a one after one
                }
                InitLevel::High => {
                    false //  a zero after zero
                }
            };
            //  strip complementary half-period part
            match wire_state_periods.pop() {
                Some(pop_period) => {
                    //  check if period is of correct value of around one resolution:
                    if 100 * pop_period > 105 * resolution {
                        //  if the period is longer it suggest it is the last period
                        //  more detailed \ build up: if zero is last encoded: this is error. zero
                        //  would cause exact period and additional random one after that.
                        match wire_state_periods.is_empty() {
                            true => {
                                break;
                            }
                            false => {
                                return Err(CodingError::FinishExpectedError);
                            }
                        }
                    } else if 100 * pop_period < 95 * resolution {
                        //  because we expect at least a finished cycle
                        return Err(CodingError::PeriodEncodingError);
                    } else {
                        //  else: the length is withing range -> push & continue
                        output.push(output_candidate).unwrap(); //  handle error
                    }
                }
                _ => {
                    //  End of the capture as there is no more elements?
                    break;
                }
            }
        }
        //  Check if the period is of value around two times of the resolution:
        //  which also means that we already land in the middle of the next bit
        else if (100 * pop_period > 190 * resolution) && (100 * pop_period < 210 * resolution) {
            let (replace_level, new_level) = match level {
                InitLevel::Low => {
                    (InitLevel::High, false) //  a zero after one
                }
                InitLevel::High => {
                    (InitLevel::Low, true) //  a one after zero
                }
            };
            level = replace_level;
            match output.push(new_level) {
                Err(_returned_element) => {
                    //  returns back elemeent that was not possible
                    return Err(CodingError::NoSpaceAvailable);
                }
                _ => { // correct
                }
            }
        } else {
            //  TODO: check for number of elements in the queue that left
            if wire_state_periods.is_empty() {
                break;
            } else {
                //  Half way through the bit there is non-conforming period length:
                return Err(CodingError::PeriodEncodingError);
            }
        }
    }
    Ok(output)
}

//  pub fn manchester_encode<
//      const N: usize, /*Vec max size*/
//      const T: usize, /*period duration in us which specifies smalles amount of time the linee can state in one state*/
//  >(
//      _level: InitLevel,
//      mut input: Vec<bool, N>,
//  ) -> Result<Vec<bool, N>, CodingError> {
//      let mut output = Vec::<bool, N>::new();
//      while let Some(element) = input.pop() {
//          match output.push(element) {
//              Err(_returned_element) => {
//                  //  returns back elemeent that was not possible
//                  return Err(CodingError::NoSpaceAvailable);
//              }
//              _ => { // correct
//              }
//          }
//          match output.push(!element) {
//              Err(_returned_element) => {
//                  //  returns back elemeent that was not possible
//                  return Err(CodingError::NoSpaceAvailable);
//              }
//              _ => { // correct
//              }
//          }
//      }
//      Ok(output)
//  }

//  #[cfg(test)]
//  mod tests {
//  //  TODO: Write some tests
//  //
//  //  test decoder of the duration table into manchaster sync protocol (check in WIO project)
//  //
//      const VEC_SIZE: usize = 64;
//      use super::*;
//      #[test]
//      fn manchester_encode() {
//          let mut timestamps = Vec::<usize, VEC_SIZE>::new();
//          timestamps.push(2_000 as usize);
//          timestamps.push(3_000 as usize);
//          timestamps.push(4_000 as usize);
//          timestamps.push(5_000 as usize);
//          ManchesterDecode::<VEC_SIZE, 1_000>(InitLevel::Low, timestamps, );
//          ()
//      }
//  }
