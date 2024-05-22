use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{debug, info};
use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};

use generic::atomi_error::AtomiError;

use crate::common::global_timer::AsyncDelay;
use crate::common::state::{LinearMotionState, MotionState};
use crate::common::stepper::classic_stepper_driver::StepperDriver;
use crate::common::stepper::GLOBAL_STEPPER_STOP;

//const FAST_SPEED: u32 = 400; // steps / second
pub const FAST_SPEED: u32 = 400; // steps / second
const SLOW_SPEED: u32 = 50; // steps / second
const SMALL_DISTANCE: i32 = 50; // steps

// TODO(zephyr): 关于stepper，我打算改成这样：
//   - 分成两类，以Trait区分：ClassicStepperDriver / SilentStepperDriver
//   - 然后有两个具体struct实现的话，其实引入到不同的环境里也比较方便。
// 这个事情要尽快做，因为后面用stepper的机会挺多的。
pub struct Stepper<
    IP1: InputPin,
    IP2: InputPin,
    OP1: StatefulOutputPin,
    OP2: OutputPin,
    OP3: OutputPin,
    D: AsyncDelay,
> {
    stepper: StepperDriver<OP1, OP2, OP3, D>,
    limit_left: IP1,
    // limit switch on the forward direction
    limit_right: IP2, // limit switch on the backward direction

    is_home: AtomicBool,
    current_position: i32,
    state: MotionState,
}

impl<IP1, IP2, OP1, OP2, OP3, D> Stepper<IP1, IP2, OP1, OP2, OP3, D>
where
    IP1: InputPin,
    IP2: InputPin,
    OP1: StatefulOutputPin,
    OP2: OutputPin,
    OP3: OutputPin,
    D: AsyncDelay,
{
    pub fn new(
        mut stepper: StepperDriver<OP1, OP2, OP3, D>,
        limit_left: IP1,
        limit_right: IP2,
    ) -> Self {
        // TODO(zephyr): 问问Lv步进电机一直使能会不会有问题。
        // 这里会有问题，首次上电会乱跑，此外一直使能，电流也会比较大，电机和驱动都会发烫。先禁用使能直到需要使用的时候再打开。
        // stepper
        //     .enable()
        //     .expect("[MMD] Errors in enable the stepper on linear actuator");
        stepper.disable().expect("[MMD] Errors in disable the stepper on linear actuator");

        Stepper {
            stepper,
            limit_left,
            limit_right,
            is_home: AtomicBool::new(false),
            current_position: 0,
            state: MotionState::new(),
        }
    }

    pub fn is_idle(&mut self) -> bool {
        info!("cur state: {}", self.state);
        self.state.is_idle()
    }

    pub fn disable(&mut self) -> Result<(), AtomiError> {
        self.is_home.store(false, Ordering::Relaxed);
        self.stepper.disable()
    }

    pub async fn home(&mut self) -> Result<i32, AtomiError> {
        // Start to re-home
        self.is_home.store(false, Ordering::Relaxed);

        self.state.push(LinearMotionState::HOMING)?;
        debug!("homing start, state: {}", self.state);
        // 第一步：快速向前直到触发限位开关
        debug!("[MMD] Home 1: move to max with fast speed till reach the limit");
        let mut steps =
            self.move_to_relative_internal(FAST_SPEED, -i32::MAX).await.map_err(|e| {
                self.state.pop();
                e
            })?;

        // 第二步：快速向后退移动一小段距离
        debug!("[MMD] Home 2: last move {} steps, now move small distance back", steps);
        steps = self.move_to_relative_internal(FAST_SPEED, SMALL_DISTANCE).await.map_err(|e| {
            self.state.pop();
            e
        })?;

        debug!(
            "[MMD] Home 3: last move {} steps, now move to the limit again with slow speed",
            steps
        );
        // 第三步：慢速向前进直到再次触发限位开关
        steps = self.move_to_relative_internal(SLOW_SPEED, -i32::MAX).await.map_err(|e| {
            self.state.pop();
            e
        })?;

        info!("[MMD] Done for home, last move {} steps", steps);
        // 将当前位置设置为 0
        self.current_position = 0;
        self.is_home.store(true, Ordering::Relaxed);

        self.state.pop();
        debug!("homing done, state: {}", self.state);
        Ok(0)
    }

    fn check_homed(&self) -> Result<(), AtomiError> {
        if !self.is_home.load(Ordering::Relaxed) {
            return Err(AtomiError::MmdStepperNeedToHome);
        }
        Ok(())
    }

    pub async fn move_to_relative(&mut self, steps: i32, speed: u32) -> Result<i32, AtomiError> {
        self.check_homed()?;
        self.move_to_relative_internal(speed, steps).await
    }

    pub async fn move_to_relative_by_force(
        &mut self,
        steps: i32,
        speed: u32,
    ) -> Result<i32, AtomiError> {
        self.move_to_relative_internal(speed, steps).await
    }

    pub async fn move_to(&mut self, position: i32, speed: u32) -> Result<i32, AtomiError> {
        if position < 0 {
            return Err(AtomiError::MmdNotAcceptedPosition);
        }
        self.check_homed()?;
        info!(
            "[MMD] current pos = {}, to pos: {}, relative: {}",
            self.current_position,
            position,
            position - self.current_position
        );
        self.move_to_relative_internal(speed, position - self.current_position).await
    }

    fn update_position_and_return(&mut self, delta: i32) -> Result<i32, AtomiError> {
        debug!(
            "[MMD] current pos: {}, new pos: {}",
            self.current_position,
            self.current_position + delta
        );
        self.current_position += delta;
        Ok(delta)
    }

    async fn move_to_relative_internal(
        &mut self,
        speed: u32,
        steps: i32,
    ) -> Result<i32, AtomiError> {
        self.stepper.ensure_enable()?;

        self.state.push(LinearMotionState::MOVING)?;
        debug!("move_to_relative_internal() 1: moving start, state: {}", self.state);

        let moving_right = steps > 0;
        self.stepper.set_speed(speed);
        self.stepper.set_direction(moving_right).map_err(|e| {
            self.state.pop();
            e
        })?;
        debug!("move_to_relative_internal() 2, steps: {}", steps);
        for i in 0..steps.abs() {
            if GLOBAL_STEPPER_STOP.load(Ordering::Relaxed) {
                return Err(AtomiError::MmdStopped);
            }
            // let l = false;
            let l = self.limit_left.is_high().unwrap_or(false);
            let r = self.limit_right.is_high().unwrap_or(false);
            // debug!(
            //     "move_to_relative_internal() 2.5: moving_right = {}, limit_left = {}, limit_right = {}",
            //     moving_right, l, r
            // );
            // 在每一步之前检查限位开关
            if moving_right && r {
                self.state.pop();
                return self.update_position_and_return(i);
            } else if !moving_right && l {
                self.state.pop();
                return self.update_position_and_return(-i);
            }
            self.stepper.step().await.map_err(|e| {
                self.state.pop();
                e
            })?;
        }
        debug!("move_to_relative_internal() 3");
        let result = self.update_position_and_return(steps);
        self.state.pop();
        debug!("moving done, state: {}", self.state);
        result
    }

    pub fn get_limit_status(&mut self) -> (bool, bool) {
        let l = self.limit_left.is_high().unwrap_or(false);
        let r = self.limit_right.is_high().unwrap_or(false);
        (l, r)
    }
}
