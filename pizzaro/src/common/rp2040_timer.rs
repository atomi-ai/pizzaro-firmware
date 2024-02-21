use rp2040_hal::Timer;

use crate::common::global_timer::{AtomiInstant, AtomiTimer};

pub struct Rp2040Timer(Timer);

impl Rp2040Timer {
    pub fn new(timer: Timer) -> Self {
        Rp2040Timer(timer)
    }
}

impl AtomiTimer for Rp2040Timer {
    fn now(&self) -> AtomiInstant {
        self.0.get_counter()
    }
}
