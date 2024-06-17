use core::sync::atomic::Ordering;
use defmt::{debug, info, Debug2Format};
use embedded_hal::digital::StatefulOutputPin;
use fugit::ExtU64;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{LinearBullCommand, LinearBullResponse};
use rp2040_hal::pwm::SliceId;

use crate::bsp::config::LINEAR_BULL;
use crate::common::brush_motor::BrushMotor;
use crate::common::global_timer::{now, Delay};
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
use crate::common::state::{LinearMotionState, MotionState};
use crate::hpd::hpd_misc::{LinearBullDirection, LinearScale};
use crate::hpd::pid::PIDController;
use crate::hpd::GLOBAL_LINEAR_BULL_STOP;

static mut LINEAR_BULL_INPUT_MQ_ONCE: Once<MessageQueueWrapper<LinearBullCommand>> = Once::new();
static mut LINEAR_BULL_OUTPUT_MQ_ONCE: Once<MessageQueueWrapper<LinearBullResponse>> = Once::new();
pub fn linear_bull_input_mq() -> &'static mut MessageQueueWrapper<LinearBullCommand> {
    unsafe { LINEAR_BULL_INPUT_MQ_ONCE.get_mut() }
}
pub fn linear_bull_output_mq() -> &'static mut MessageQueueWrapper<LinearBullResponse> {
    unsafe { LINEAR_BULL_OUTPUT_MQ_ONCE.get_mut() }
}

pub async fn process_linear_bull_message<S: SliceId, E: StatefulOutputPin>(
    mut processor: LinearBullProcessor<S, E>,
) {
    debug!("process_linear_bull_message() 0");
    let mq_in = linear_bull_input_mq();
    let mq_out = linear_bull_output_mq();
    loop {
        if let Some(msg) = mq_in.dequeue() {
            info!("process_linear_bull_message() 3.1: process msg {}", Debug2Format(&msg));
            let resp = processor
                .process_linear_bull_message(msg)
                .await
                .unwrap_or_else(LinearBullResponse::Error);
            info!("process_linear_bull_message() 3.3: done, resp = {}", Debug2Format(&resp));
            mq_out.enqueue(resp);
        }
        Delay::new(1.millis()).await;
    }
}

pub struct LinearBullProcessor<S: SliceId, E: StatefulOutputPin> {
    linear_scale: &'static mut LinearScale,
    pwm_motor: BrushMotor<S, E>,
    state: MotionState,
}

impl<S: SliceId, E: StatefulOutputPin> LinearBullProcessor<S, E> {
    pub fn new(linear_scale: &'static mut LinearScale, pwm_motor: BrushMotor<S, E>) -> Self {
        Self { linear_scale, pwm_motor, state: MotionState::new() }
    }

    pub fn is_idle(&mut self) -> bool {
        self.state.is_idle()
    }

    pub async fn process_linear_bull_message<'a>(
        &mut self,
        msg: LinearBullCommand,
    ) -> Result<LinearBullResponse, AtomiError> {
        info!("process_linear_bull_message() 0: msg: {}", Debug2Format(&msg));
        match msg {
            LinearBullCommand::Home => {
                Ok(LinearBullResponse::Position { position: self.home().await? })
            }
            LinearBullCommand::MoveTo { position } => {
                Ok(LinearBullResponse::Position { position: self.move_to(position).await? })
            }
            LinearBullCommand::MoveToRelative { distance } => Ok(LinearBullResponse::Position {
                position: self.move_to_relative(distance).await?,
            }),
            LinearBullCommand::WaitIdle => {
                while !self.is_idle() {
                    let _ = Delay::new(10.millis()).await;
                }
                Ok(LinearBullResponse::Done)
            }
            LinearBullCommand::DummyWait { seconds } => {
                let _ = Delay::new((seconds as u64).secs()).await;
                Ok(LinearBullResponse::Done)
            }
            LinearBullCommand::GetPosition => {
                let position = self.linear_scale.get_abs_position()?;
                info!("xfguo: process_linear_bull_message() 3.5, pos = {}", position);
                Ok(LinearBullResponse::Position { position })
            }
        }
    }

    fn check_homed(&self) -> Result<i32, AtomiError> {
        self.linear_scale.check_homed()
    }

    pub async fn home(&mut self) -> Result<i32, AtomiError> {
        self.pwm_motor.enable()?;

        let dir = LinearBullDirection::Bottom;
        info!(
            "xfguo: Home to one direction, dir: {}, from: {}, to: {}",
            dir,
            Debug2Format(&self.linear_scale.get_rel_position()),
            dir.get_most_position()
        );
        self.home_on_direction(dir).await?;
        info!("xfguo: After homed, linear_scale = {}", Debug2Format(self.linear_scale));
        self.linear_scale.set_home();
        info!("xfguo: After homed 2, linear_scale = {}", Debug2Format(self.linear_scale));
        self.linear_scale.get_abs_position()
    }

    async fn home_on_direction(&mut self, dir: LinearBullDirection) -> Result<i32, AtomiError> {
        self.state.push(LinearMotionState::HOMING)?;
        info!("home_on_direction 0, now = {}", now().ticks());
        let t = self.move_with_speed(dir.get_dir_seg() * 1.0, 10_000_000).await;
        // .map_err(|e| {
        self.state.pop();
        //     e
        // })?;
        // info!("home_on_direction 2, now = {}", now().ticks());
        // self.move_with_speed(dir.get_dir_seg() * -1.0, 1000).await.map_err(|e| {
        //     self.state.pop();
        //     e
        // })?;
        // info!("home_on_direction 4, now = {}", now().ticks());
        // let t = self.move_with_speed(dir.get_dir_seg() * 1.0, 10_000_000).await;
        // self.state.pop();
        // info!("home_on_direction 9, now = {}", now().ticks());
        t
    }

    async fn move_with_speed(&mut self, speed: f32, repeat_times: i32) -> Result<i32, AtomiError> {
        debug!(
            "LinearBullProcessor::move_with_speed(): speed = {}, repeat_times = {}",
            speed, repeat_times
        );
        self.linear_scale.reset_stationary();
        for _ in 0..repeat_times {
            Delay::new(1.millis()).await;
            if GLOBAL_LINEAR_BULL_STOP.load(Ordering::Relaxed) {
                // info!("stop flag got");
                self.pwm_motor.apply_speed(0.0);
                return Err(AtomiError::HpdStopped);
            }
            if self.linear_scale.is_stationary() {
                break;
            }
            //info!("pos = {}", self.linear_scale.get_rel_position());
            self.pwm_motor.apply_speed(speed);
        }
        self.pwm_motor.apply_speed(0.0);
        self.linear_scale.get_rel_position()
    }

    /// Arguments:
    ///   - distance: unit in 0.01mm
    pub async fn move_to_relative(&mut self, distance: i32) -> Result<i32, AtomiError> {
        info!("xfguo: move_to_relative() 0, distance: {}", distance);
        let target = self.linear_scale.get_rel_position()? + distance;
        info!("xfguo: move_to_relative() 1, target: {}", target);
        let mut pid = PIDController::new(LINEAR_BULL.0, LINEAR_BULL.1, LINEAR_BULL.2, target);
        let dt = 0.01;
        self.linear_scale.reset_stationary();

        self.state.push(LinearMotionState::MOVING)?;
        let mut stationary_counter = 0u8;
        loop {
            Delay::new(1.millis()).await;
            if GLOBAL_LINEAR_BULL_STOP.load(Ordering::Relaxed) {
                // info!("stop flag got");
                self.pwm_motor.apply_speed(0.0);
                return Err(AtomiError::HpdStopped);
            }
            let pos = self.linear_scale.get_rel_position().map_err(|e| {
                self.state.pop();
                e
            })?;
            if pid.reach_target(pos, now()) {
                break;
            }
            stationary_counter = stationary_counter.wrapping_add(1);
            if stationary_counter & 0xf == 1 && self.linear_scale.is_stationary_2() {
                break;
            }

            // Only trigger PID when the position is changed.
            let speed = pid.calculate(pos, dt);
            info!("Current pos: {}, speed = {}, target = {}", pos, speed, target);
            self.pwm_motor.apply_speed_freerun(speed, false);
        }
        self.pwm_motor.apply_speed(0.0);
        self.state.pop();
        self.linear_scale.get_rel_position()
    }

    pub async fn move_to(&mut self, target_position: i32) -> Result<i32, AtomiError> {
        debug!("LinearBullProcessor::move_to() 0: target_position: {}", target_position);
        self.check_homed()?;
        debug!("LinearBullProcessor::move_to() 1: target_position: {}", target_position);
        self.move_to_relative(self.linear_scale.get_distance(target_position)?).await
    }
}
