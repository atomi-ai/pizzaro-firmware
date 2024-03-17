// TODO(zephyr): Move the file to ../testing?

use core::fmt::Debug;

use defmt::info;
use fugit::ExtU64;

use pizzaro::common::global_status::{set_status, FutureStatus, FutureType};
use pizzaro::common::global_timer::{now, Delay};

use crate::mock_stepper::x::StepperAdapter;

#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum Direction {
    Left,
    Right,
}

fn direction_to_seg(dir: Direction) -> i64 {
    match dir {
        Direction::Left => -1,
        Direction::Right => 1,
    }
}

#[derive(Copy, Clone, PartialEq, Debug, defmt::Format)]
pub enum StepperState {
    OnMove,
    Stay,
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct LimitSwitch {
    triggered: bool,
    position: i64, // um
}

impl LimitSwitch {
    fn new(position: i64) -> Self {
        LimitSwitch {
            triggered: false,
            position,
        }
    }

    pub fn is_triggered(&self) -> bool {
        self.triggered
    }

    fn position_update(&mut self, pos: i64) {
        if pos != self.position {
            self.triggered = false;
        } else {
            self.triggered = true;
        }
    }
}

// TODO(zephyr): trait Stepper should not have move_to_relative() and others(), this should
//     be in LinearActuator.
pub trait Stepper: defmt::Format {
    fn tick(&mut self);
    fn move_stepper(&mut self, direction: Direction, speed: i64);
    fn reset_position(&mut self);

    // TODO(zephyr)
    fn move_to_relative(&mut self, direction: Direction, distance: i64, speed: i64);
    fn get_state(&self) -> StepperState;
}

// TODO(zephyr): Split this as "trait Stepper" and "LinearActuator { stepper, limit, destination...}"
#[derive(Debug, defmt::Format)]
pub struct MockStepper {
    position: i64, // um
    pub left_limit: LimitSwitch,
    pub right_limit: LimitSwitch,
    state: StepperState,
    direction: Direction,
    speed: i64, // um / step
    destination: i64,
}

impl MockStepper {
    pub fn new(left_limit_pos: i64, right_limit_pos: i64) -> Self {
        MockStepper {
            position: 0,
            left_limit: LimitSwitch::new(left_limit_pos),
            right_limit: LimitSwitch::new(right_limit_pos),
            state: StepperState::Stay,
            direction: Direction::Right,
            speed: 0,
            destination: i64::MIN,
        }
    }

    fn check_stop_or_move(&mut self, limit: LimitSwitch) {
        let seg = direction_to_seg(self.direction);
        if limit.triggered
            || seg * self.position >= seg * limit.position
            || (self.destination > i64::MIN && seg * self.position >= seg * self.destination)
        {
            self.state = StepperState::Stay;
            return;
        }
        self.position += seg * self.speed;
    }
}

impl Stepper for MockStepper {
    fn tick(&mut self) {
        if matches!(self.state, StepperState::Stay) {
            // do nothing
            return;
        }

        // StepperState::OnMove
        match self.direction {
            Direction::Left => self.check_stop_or_move(self.left_limit),
            Direction::Right => self.check_stop_or_move(self.right_limit),
        }
        self.left_limit.position_update(self.position);
        self.right_limit.position_update(self.position);
    }

    fn move_stepper(&mut self, direction: Direction, speed: i64) {
        self.direction = direction;
        self.speed = speed;
        self.state = StepperState::OnMove;
    }

    fn reset_position(&mut self) {
        let (l, r) = (
            self.left_limit.position - self.position,
            self.right_limit.position - self.position,
        );
        self.right_limit = LimitSwitch::new(r);
        self.left_limit = LimitSwitch::new(l);
        self.position = 0;
    }

    fn move_to_relative(&mut self, direction: Direction, distance: i64, speed: i64) {
        self.state = StepperState::OnMove;
        self.direction = direction;
        self.speed = speed;
        self.destination = self.position + distance;
    }

    fn get_state(&self) -> StepperState {
        self.state
    }
}

pub async fn stepper_tick(mut stepper: StepperAdapter<MockStepper>) {
    loop {
        stepper.tick();
        info!(
            "stepper_tick() 1.5 After one tick: {:?} | ts: {:?}",
            stepper,
            now().ticks()
        );
        Delay::new(100.millis()).await;
        // info!("stepper_tick() 1.9 after wait: | ts: {:?}", now().ticks());
    }
}

pub async fn wait_to_finish<S: Stepper + defmt::Format>(stepper: &S) {
    loop {
        // info!("wait_to_finish() 1.0: {:?} | {:?}", stepper, now().ticks());
        if stepper.get_state() == StepperState::Stay {
            return;
        }
        Delay::new(100.millis()).await;
        // info!("wait_to_finish() 1.9 after delay: {:?}", now().ticks());
    }
}

pub async fn homing<S: Stepper + defmt::Format>(mut stepper: S, fast_speed: i64, slow_speed: i64) {
    info!("Stepper start homing: {:?}", stepper);

    // 第一阶段：快速左移直到触发限位
    info!("Move left to MIN");
    stepper.move_to_relative(Direction::Left, i64::MIN, fast_speed);
    set_status(FutureType::StepperHoming, FutureStatus::HomingStage1);
    wait_to_finish(&stepper).await;

    // 第二阶段：稍微右移一点
    info!("Move right for 3 steps");
    stepper.move_to_relative(Direction::Right, fast_speed, fast_speed);
    set_status(FutureType::StepperHoming, FutureStatus::HomingStage2);
    wait_to_finish(&stepper).await;

    // 第三阶段：慢速左移直到再次触发限位
    info!("Move left again");
    stepper.move_to_relative(Direction::Left, -fast_speed, slow_speed);
    set_status(FutureType::StepperHoming, FutureStatus::HomingStage3);
    wait_to_finish(&stepper).await;

    set_status(FutureType::StepperHoming, FutureStatus::HomingDone);
    stepper.reset_position();
    set_status(FutureType::StepperHoming, FutureStatus::Completed);
}

pub mod x {
    use core::fmt::Debug;

    use crate::mock_stepper::{Direction, Stepper, StepperState};

    #[derive(Debug, defmt::Format)]
    pub struct StepperAdapter<T: 'static + Stepper>(pub &'static mut T);

    impl<T: 'static + Stepper> Stepper for StepperAdapter<T> {
        fn tick(&mut self) {
            critical_section::with(|_cs| self.0.tick())
        }

        fn move_stepper(&mut self, direction: Direction, speed: i64) {
            critical_section::with(|_cs| self.0.move_stepper(direction, speed))
        }

        fn reset_position(&mut self) {
            critical_section::with(|_cs| self.0.reset_position())
        }

        fn move_to_relative(&mut self, direction: Direction, distance: i64, speed: i64) {
            critical_section::with(|_cs| self.0.move_to_relative(direction, distance, speed))
        }

        fn get_state(&self) -> StepperState {
            critical_section::with(|_cs| self.0.get_state())
        }
    }
}
