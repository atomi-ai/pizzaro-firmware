use defmt::{Debug2Format, info};
use fugit::ExtU64;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{LinearBullCommand, LinearBullResponse};

use crate::bsp::config::LINEAR_BULL;
use crate::common::global_timer::{Delay, now};
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
use crate::hpd::hpd_misc::{LinearBullDirection, LinearScale, PwmMotor};
use crate::hpd::pid::PIDController;

static mut LINEAR_BULL_INPUT_MQ_ONCE: Once<MessageQueueWrapper<LinearBullCommand>> = Once::new();
static mut LINEAR_BULL_OUTPUT_MQ_ONCE: Once<MessageQueueWrapper<LinearBullResponse>> = Once::new();
pub fn linear_bull_input_mq() -> &'static mut MessageQueueWrapper<LinearBullCommand> {
    unsafe { LINEAR_BULL_INPUT_MQ_ONCE.get_mut() }
}
pub fn linear_bull_output_mq() -> &'static mut MessageQueueWrapper<LinearBullResponse> {
    unsafe { LINEAR_BULL_OUTPUT_MQ_ONCE.get_mut() }
}

pub async fn process_linear_bull_message(mut processor: LinearBullProcessor) {
    info!("process_linear_bull_message() 0");
    let mq_in = linear_bull_input_mq();
    let mq_out = linear_bull_output_mq();
    loop {
        if let Some(msg) = mq_in.dequeue() {
            info!("process_linear_bull_message() 3.1: process msg {}", msg);
            let res = match processor.process_linear_bull_message(msg).await {
                Ok(_) => LinearBullResponse::Done,
                Err(err) => LinearBullResponse::Error(err),
            };
            info!("process_linear_bull_message() 3.3: done");
            mq_out.enqueue(res);
        }
        Delay::new(1.millis()).await;
    }
}

pub struct LinearBullProcessor {
    linear_scale: &'static mut LinearScale,
    pwm_motor: PwmMotor,
}

impl LinearBullProcessor {
    pub fn new(linear_scale: &'static mut LinearScale, pwm_motor: PwmMotor) -> Self {
        Self {
            linear_scale,
            pwm_motor,
        }
    }

    pub async fn process_linear_bull_message<'a>(&mut self, msg: LinearBullCommand) -> Result<i32, AtomiError> {
        match msg {
            LinearBullCommand::Home => self.home().await,
            LinearBullCommand::MoveTo { position } => self.move_to(position).await,
            LinearBullCommand::MoveToRelative { distance } => self.move_to_relative(distance).await,
            LinearBullCommand::DummyWait { seconds } => {
                let _ = Delay::new((seconds as u64).secs()).await;
                Ok(0)
            }
        }
    }

    fn check_homed(&self) -> Result<i32, AtomiError> {
        self.linear_scale.check_homed()
    }

    pub async fn home(&mut self) -> Result<i32, AtomiError> {
        self.pwm_motor.start_pwm_motor()?;

        let dir = LinearBullDirection::Bottom;
        info!("xfguo: Home to one direction, dir: {}, from: {}, to: {}",
            dir, self.linear_scale.get_rel_position(), dir.get_most_position());
        self.home_on_direction(dir).await?;
        info!("xfguo: After homed, linear_scale = {}", Debug2Format(self.linear_scale));
        self.linear_scale.set_home();
        info!("xfguo: After homed 2, linear_scale = {}", Debug2Format(self.linear_scale));
        self.linear_scale.get_abs_position()
    }

    async fn home_on_direction(&mut self, dir: LinearBullDirection) -> Result<i32, AtomiError> {
        info!("home_on_direction 0, now = {}", now().ticks());
        self.move_with_speed(dir.get_dir_seg() * 1.0, 10_000_000)
            .await?;
        info!("home_on_direction 2, now = {}", now().ticks());
        self.move_with_speed(dir.get_dir_seg() * -1.0, 1000).await?;
        info!("home_on_direction 4, now = {}", now().ticks());
        let t = self
            .move_with_speed(dir.get_dir_seg() * 1.0, 10_000_000)
            .await;
        info!("home_on_direction 9, now = {}", now().ticks());
        t
    }

    async fn move_with_speed(&mut self, speed: f32, repeat_times: i32) -> Result<i32, AtomiError> {
        info!("xfguo: speed = {}, repeat_times = {}", speed, repeat_times);
        self.linear_scale.reset_stationary();
        for _ in 0..repeat_times {
            Delay::new(1.millis()).await;
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
        let target = self.linear_scale.get_rel_position()? + distance;
        info!("xfguo: move_to_relative() 1, target: {}", target);
        let mut pid = PIDController::new(LINEAR_BULL.0, LINEAR_BULL.1, LINEAR_BULL.2, target);
        let dt = 0.01;

        loop {
            Delay::new(1.millis()).await;
            let pos = self.linear_scale.get_rel_position()?;
            if pid.reach_target(pos, now()) {
                break;
            }
            // if self.linear_scale.is_stationary() {
            //     break;
            // }
            // Only trigger PID when the position is changed.
            let speed = pid.calculate(pos, dt);
            info!(
                "Current pos: {}, speed = {}, target = {}",
                pos, speed, target
            );
            self.pwm_motor.apply_speed(speed);
        }
        self.pwm_motor.apply_speed(0.0);
        self.linear_scale.get_rel_position()
    }

    pub async fn move_to(&mut self, target_position: i32) -> Result<i32, AtomiError> {
        self.check_homed()?;
        self.move_to_relative(self.linear_scale.get_distance(target_position)?)
            .await
    }
}
