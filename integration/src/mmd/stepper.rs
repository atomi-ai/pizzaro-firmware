use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU64;

use common::global_timer::AsyncDelay;
use generic::atomi_error::AtomiError;

pub struct Stepper<OP1: OutputPin, OP2: OutputPin, OP3: OutputPin, D: AsyncDelay> {
    enable_pin: OP1,
    dir_pin: OP2,
    step_pin: OP3,

    speed: u32,
    wait_period: u64,
    async_delay: D,
}

impl<OP1: OutputPin, OP2: OutputPin, OP3: OutputPin, D: AsyncDelay> Stepper<OP1, OP2, OP3, D> {
    pub fn new(enable_pin: OP1, dir_pin: OP2, step_pin: OP3, async_delay: D) -> Self {
        Stepper {
            enable_pin,
            dir_pin,
            step_pin,
            async_delay,
            speed: 0,
            wait_period: 0,
        }
    }

    pub fn enable(&mut self) -> Result<(), AtomiError> {
        self.enable_pin.set_low().map_err(|_| AtomiError::MmdPinError)
    }

    pub fn disable(&mut self) -> Result<(), AtomiError> {
        self.enable_pin.set_high().map_err(|_| AtomiError::MmdPinError)
    }

    pub fn set_direction(&mut self, forward: bool) -> Result<(), AtomiError> {
        if forward {
            self.dir_pin.set_high().map_err(|_| AtomiError::MmdPinError)?;
        } else {
            self.dir_pin.set_low().map_err(|_| AtomiError::MmdPinError)?;
        }
        Ok(())
    }

    pub fn set_speed(&mut self, speed: u32) {
        self.speed = speed;
        if self.speed != 0 {
            self.wait_period = (1_000_000 / (speed * 2)) as u64;
        }
    }

    pub async fn step(&mut self) -> Result<(), AtomiError> {
        if self.speed == 0 {
            return Err(AtomiError::MmdMoveWithZeroSpeed);
        }
        self.step_pin.set_high().map_err(|_| AtomiError::MmdPinError)?;
        self.async_delay.delay(self.wait_period.micros()).await;
        self.step_pin.set_low().map_err(|_| AtomiError::MmdPinError)?;
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
