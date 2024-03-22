use core::sync::atomic::AtomicBool;

pub mod brush_motor_processor;
pub mod brushless_motor_processor;
pub mod linear_stepper;
pub mod linear_stepper_processor;
pub mod rotation_stepper_processor;
#[allow(async_fn_in_trait)]
pub mod stepper;

pub static GLOBAL_LINEAR_STEPPER_STOP: AtomicBool = AtomicBool::new(false);
