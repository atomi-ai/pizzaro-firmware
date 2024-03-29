extern crate alloc;

use crate::common::global_allocator::init_allocator;
use crate::common::global_status::initialize_status;

pub mod consts;
pub mod executor;
pub mod global_allocator;
pub mod global_status;
pub mod global_timer;
pub mod led_controller;
pub mod message_queue;
pub mod once;
pub mod rp2040_timer;
pub mod tmc2209;
pub mod tmc5160;
pub mod uart;
pub mod uart_comm;

pub fn async_initialization() {
    init_allocator();
    initialize_status();

    // You may also need to initialize global_timer, but it depends on
    // having a timer. You can do it later by yourself, as below:
    //
    // init_global_timer(Box::new(Rp2040Timer::new(timer)));
}
