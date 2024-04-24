use core::sync::atomic::AtomicBool;

pub mod brush_motor_processor;
pub mod brushless_motor_processor;
pub mod rotation_stepper_processor;
pub mod stepper;
pub mod stepper_processor;

pub static GLOBAL_STEPPER_STOP: AtomicBool = AtomicBool::new(false);
