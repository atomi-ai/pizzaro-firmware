use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
use fugit::ExtU64;

use generic::atomi_error::AtomiError;

use crate::common::global_timer::AsyncDelay;

pub struct Stepper<OP1: StatefulOutputPin, OP2: OutputPin, OP3: OutputPin, D: AsyncDelay> {
    enable_pin: OP1,
    dir_pin: OP2,
    step_pin: OP3,
    revert_dir: bool,

    speed: u32,
    wait_period: u64,
    async_delay: D,
}

impl<OP1: StatefulOutputPin, OP2: OutputPin, OP3: OutputPin, D: AsyncDelay>
    Stepper<OP1, OP2, OP3, D>
{
    pub fn new(
        enable_pin: OP1,
        dir_pin: OP2,
        step_pin: OP3,
        async_delay: D,
        revert_dir: bool,
    ) -> Self {
        Stepper { enable_pin, dir_pin, step_pin, revert_dir, async_delay, speed: 0, wait_period: 0 }
    }

    pub fn enable(&mut self) -> Result<(), AtomiError> {
        self.enable_pin.set_low().map_err(|_| AtomiError::GpioPinError)
    }

    pub fn disable(&mut self) -> Result<(), AtomiError> {
        self.enable_pin.set_high().map_err(|_| AtomiError::GpioPinError)
    }

    pub fn ensure_enable(&mut self) -> Result<(), AtomiError> {
        if self.enable_pin.is_set_high().map_err(|_| AtomiError::GpioPinError)? {
            self.enable()
        } else {
            Ok(())
        }
    }

    pub fn set_direction(&mut self, forward: bool) -> Result<(), AtomiError> {
        if forward ^ self.revert_dir {
            self.dir_pin.set_high().map_err(|_| AtomiError::GpioPinError)?;
        } else {
            self.dir_pin.set_low().map_err(|_| AtomiError::GpioPinError)?;
        }
        Ok(())
    }

    pub fn set_speed(&mut self, speed: u32) {
        self.speed = speed;
        if self.speed != 0 {
            self.wait_period = (1_000_000 / (speed * 2)) as u64;
        }
    }

    // TODO:set_speed with acceleration, need test
    //
    // pub fn set_speed(&mut self, speed: u32, accl: u32) {
    //     self.target_speed = speed;
    //     self.current_speed = 0;
    //     self.acceleration = accl;
    // }

    // pub fn update_current_speed(&mut self) {
    //     if self.current_speed < self.target_speed {
    //         self.current_speed = min(self.current_speed + self.acceleration, self.target_speed);
    //         if self.current_speed > 0 {
    //             self.wait_period = (1_000_000 / (self.current_speed * 2)) as u64;
    //         }
    //     }
    // }

    // pub async fn step(&mut self) -> Result<(), AtomiError> {
    //     self.update_current_speed();
    //     if self.speed == 0 {
    //         return Err(AtomiError::MmdMoveWithZeroSpeed);
    //     }
    //     self.step_pin.set_high().map_err(|_| AtomiError::GpioPinError)?;
    //     self.async_delay.delay(self.wait_period.micros()).await;
    //     self.step_pin.set_low().map_err(|_| AtomiError::GpioPinError)?;
    //     self.async_delay.delay(self.wait_period.micros()).await;
    //     Ok(())
    // }

    pub async fn step(&mut self) -> Result<(), AtomiError> {
        if self.speed == 0 {
            return Err(AtomiError::MmdMoveWithZeroSpeed);
        }
        self.step_pin.set_high().map_err(|_| AtomiError::GpioPinError)?;
        self.async_delay.delay(self.wait_period.micros()).await;
        self.step_pin.set_low().map_err(|_| AtomiError::GpioPinError)?;
        self.async_delay.delay(self.wait_period.micros()).await;
        Ok(())
    }
    //
    // pub async fn run(&mut self, steps: u32, speed: u32) -> Result<u32, T::Error> {
    //     self.should_stop.store(false, Ordering::Relaxed);
    //
    //     let wait_period = 1_000_000 / (speed * 2) as u64;
    //     for i in 0..steps {
    //         if self.should_stop.load(Ordering::Relaxed) {
    //             return Ok(i);
    //         }
    //         self.step_pin.set_high()?;
    //         self.async_delay.delay(wait_period.micros()).await;
    //         self.step_pin.set_low()?;
    //         self.async_delay.delay(wait_period.micros()).await;
    //     }
    //     Ok(steps)
    // }
    //
    // pub fn stop(&mut self) {
    //     self.should_stop.store(true, Ordering::Relaxed)
    // }
}
