use embassy_time::Duration;
use heapless::Vec;

#[derive(Clone, PartialEq)]
pub enum InitLevel {
    Low,
    High,
}

pub enum TriggerError {
    GenericError,
}

pub enum CaptureError {
    GenericError,
    InvalidData,
    NotEnoughSpace,
}

pub const VEC_DEFAULT_SIZE_MANCHASTER: usize = 128usize;

pub enum FinishState {
    IdleState,
    TimeoutReached,
    CountReached,
}

pub trait EdgeTriggerInterface {
    async fn trigger(&mut self, iterator: impl Iterator<Item = bool>) -> Result<(), TriggerError>;
}

pub trait EdgeCaptureInterface<const N: usize = VEC_DEFAULT_SIZE_MANCHASTER> {
    //  TODO: specify /generically/ idle level or maybe better, return one captured:
    async fn start_capture(
        &mut self,
        timeout_inactive_capture: Duration,
        timeout_till_active_capture: Duration,
    ) -> Result<(InitLevel, Vec<Duration, N>), CaptureError>;
}
