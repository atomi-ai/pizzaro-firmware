use core::sync::atomic::AtomicBool;

#[allow(async_fn_in_trait)]
pub mod tmc_driver;
pub mod tmc_stepper;
pub mod tmc_stepper_processor;

pub static GLOBAL_DTU_STEPPER_STOP: AtomicBool = AtomicBool::new(false);
