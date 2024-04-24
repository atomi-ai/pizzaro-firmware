use core::sync::atomic::AtomicBool;

#[allow(async_fn_in_trait)]
pub mod tmc_stepper;
pub mod dtu_linear_stepper;
pub mod dtu_linear_stepper_processor;

pub static GLOBAL_DTU_LINEAR_STEPPER_STOP: AtomicBool = AtomicBool::new(false);
