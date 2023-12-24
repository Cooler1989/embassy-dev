use heapless::Vec;
//  use core::time::Duration;
use embassy_time::{Duration};

use super::InitLevel;

pub enum CodingError {
    GenericError,
    NoSpaceAvailable,
}

//  TODO: Prepare data first - remove first and last long periods
pub fn manchester_decode<const N: usize, const T: usize /*period duration in us which specifies smalles amount of time the linee can state in one state*/>(
    level: InitLevel,
    mut wire_state_periods: Vec<Duration, N>,
) -> Result<Vec<bool, N>, CodingError> {
    let mut output = Vec::<bool, N>::new();
    //  discard first cycle as it is not relevant because
    //  capture should start before first relevant level transition
    //  TODO: possibly do it upstream
    let _ignore = wire_state_periods.pop().unwrap();

    let resolution = Duration::from_micros(T as u64);
    //  Initial level is very important as it determines all other transitions
    //  TODO: make it more redundant and safe to mistake one level transition and still have most
    //  of the data correct
    let mut level = level;  // init level later used as mut to handle iterations
    loop {
        match wire_state_periods.pop() {
            Some(pop_period) => {
                if level == InitLevel::Low
                {
                    output.push(true).unwrap();
                }
                else
                {
                    output.push(false).unwrap();
                }
                //  TODO: check tolerance!
                if pop_period > resolution
                { }

            }
            _ => {
                break;
            }
        }
        //for time in wire_state_periods.iter() {
    }
    Ok(output)
}

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
