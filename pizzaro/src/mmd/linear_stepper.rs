use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use defmt::info;
use embedded_hal::digital::v2::{InputPin, OutputPin};

use generic::atomi_error::AtomiError;

use crate::common::global_timer::AsyncDelay;
use crate::mmd::stepper::Stepper;

const FAST_SPEED: u32 = 100;  // steps / second
const SLOW_SPEED: u32 = 50;  // steps / second
const SMALL_DISTANCE: i32 = 50;  // steps

pub struct LinearStepper<IP1: InputPin, IP2: InputPin, OP1: OutputPin, OP2: OutputPin, OP3: OutputPin, D: AsyncDelay> {
    stepper: Stepper<OP1, OP2, OP3, D>,
    limit_left: IP1,
    // limit switch on the forward direction
    limit_right: IP2,  // limit switch on the backward direction

    is_home: AtomicBool,
    current_position: i32,
}

impl<IP1, IP2, OP1, OP2, OP3, D> LinearStepper<IP1, IP2, OP1, OP2, OP3, D>
    where
        IP1: InputPin,
        IP2: InputPin,
        OP1: OutputPin,
        OP2: OutputPin,
        OP3: OutputPin,
        D: AsyncDelay,
{
    pub fn new(mut stepper: Stepper<OP1, OP2, OP3, D>, limit_left: IP1, limit_right: IP2) -> Self {
        // TODO(zephyr): 问问Lv步进电机一直使能会不会有问题。
        stepper.enable().expect("[MMD] Errors in enable the stepper on linear actuator");
        LinearStepper {
            stepper,
            limit_left,
            limit_right,
            is_home: AtomicBool::new(false),
            current_position: 0,
        }
    }

    pub async fn home(&mut self) -> Result<i32, AtomiError> {
        // Start to re-home
        self.is_home.store(false, Relaxed);

        // 第一步：快速向前直到触发限位开关
        info!("[MMD] Home 1: move to max with fast speed till reach the limit");
        let mut steps = self.move_to_relative_internal(FAST_SPEED, -i32::MAX).await?;

        // 第二步：快速向后退移动一小段距离
        info!("[MMD] Home 2: last move {} steps, now move small distance back", steps);
        steps = self.move_to_relative_internal(FAST_SPEED, SMALL_DISTANCE).await?;

        info!("[MMD] Home 3: last move {} steps, now move to the limit again with slow speed", steps);
        // 第三步：慢速向前进直到再次触发限位开关
        steps = self.move_to_relative_internal(SLOW_SPEED, -i32::MAX).await?;

        info!("[MMD] Done for home, last move {} steps", steps);
        // 将当前位置设置为 0
        self.current_position = 0;
        self.is_home.store(true, Relaxed);

        Ok(0)
    }

    fn check_homed(&self) -> Result<(), AtomiError> {
        if !self.is_home.load(Relaxed) {
            return Err(AtomiError::MmdStepperNeedToHome);
        }
        Ok(())
    }

    pub async fn move_to_relative(&mut self, steps: i32) -> Result<i32, AtomiError> {
        self.check_homed()?;
        self.move_to_relative_internal(FAST_SPEED, steps).await
    }

    pub async fn move_to(&mut self, position: i32) -> Result<i32, AtomiError> {
        if position < 0 {
            return Err(AtomiError::MmdNotAcceptedPosition);
        }
        self.check_homed()?;
        info!("[MMD] current pos = {}, to pos: {}, relative: {}",
            self.current_position, position, position - self.current_position);
        self.move_to_relative_internal(FAST_SPEED, position - self.current_position).await
    }

    fn update_position_and_return(&mut self, delta: i32) -> Result<i32, AtomiError> {
        info!("[MMD] current pos: {}, new pos: {}", self.current_position, self.current_position + delta);
        self.current_position += delta;
        Ok(delta)
    }

    async fn move_to_relative_internal(&mut self, speed: u32, steps: i32) -> Result<i32, AtomiError> {
        let forward = steps > 0;
        self.stepper.set_speed(speed);
        self.stepper.set_direction(forward)?;
        for i in 0..steps.abs() {
            let l = self.limit_left.is_high().unwrap_or(false);
            let r = self.limit_right.is_high().unwrap_or(false);
            // info!("[MMD: Debug] forward = {}, limit_left = {}, limit_right = {}", forward, l, r);
            // 在每一步之前检查限位开关
            if forward && l {
                return self.update_position_and_return(i);
            } else if !forward && r {
                return self.update_position_and_return(-i);
            }
            self.stepper.step().await?;
        }
        self.update_position_and_return(steps)
    }
}
