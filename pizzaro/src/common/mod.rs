// TODO(zephyr): create a atomi-common crate for usage in other repos.
extern crate alloc;

use crate::common::global_allocator::init_allocator;
use crate::common::global_status::initialize_status;

pub mod brush_motor;
pub mod brush_motor_patch;
pub mod brushless_motor;
pub mod can_messenger;
pub mod consts;
pub mod executor;
pub mod global_allocator;
pub mod global_status;
pub mod global_timer;
pub mod led_controller;
pub mod message_queue;
pub mod once;
pub mod pwm_stepper;
pub mod rp2040_timer;
pub mod state;
#[allow(async_fn_in_trait)]
pub mod stepper_driver;
#[allow(
    clippy::get_first,
    clippy::from_over_into,
    clippy::assign_op_pattern,
    clippy::approx_constant,
    clippy::manual_range_contains,
    clippy::too_many_arguments
)]
pub mod tmc2209;
pub mod uart;
pub mod uart_comm;
pub mod weight_sensor;

pub fn async_initialization() {
    init_allocator();
    initialize_status();

    // You may also need to initialize global_timer, but it depends on
    // having a timer. You can do it later by yourself, as below:
    //
    // init_global_timer(Box::new(Rp2040Timer::new(timer)));
}
