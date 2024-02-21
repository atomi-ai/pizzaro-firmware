#![no_std]
extern crate alloc;

pub mod cap_displacement_processor;
pub mod global_timer;
pub mod global_status;
pub mod global_allocator;
pub mod uart;
pub mod executor;
pub mod rp2040_timer;

pub fn async_initialization() {
    global_allocator::init_allocator();
    global_status::initialize_status();

    // You may also need to initialize global_timer, but it depends on
    // having a timer. You can do it later by yourself, as below:
    //
    // init_global_timer(Box::new(Rp2040Timer::new(timer)));
}
