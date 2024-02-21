use crate::pid::PIDController;
use common::global_timer::{now, AtomiDuration, AtomiInstant, Delay};
use core::sync::atomic::{AtomicI32, Ordering};
use defmt::{info, Debug2Format, Format};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use fugit::ExtU64;
use generic::atomi_error::AtomiError;
use rp2040_hal::gpio::bank0::Gpio18;
use rp2040_hal::gpio::{FunctionSio, Pin, PullDown, SioOutput};
use rp2040_hal::pwm::{FreeRunning, Pwm0, Slice};

const MAX_POSITION: i32 = 10_000_000;
const MIN_POSITION: i32 = -MAX_POSITION;
// const LITTLE_DISTANCE: i32 = 1000;  // 10mm
const MOTOR150_PWM_TOP: u16 = 2000;
const STATIONARY_POSITION_THRESHOLD: i32 = 10;
const STATIONARY_TIME_THRESHOLD: AtomiDuration = AtomiDuration::millis(200);

#[derive(Copy, Clone, Debug, Format)]
enum HpdDirection {
    _Top,  // Add "Top" back if necessary.
    Bottom,
}

impl HpdDirection {
    fn get_most_position(&self) -> i32 {
        match self {
            HpdDirection::_Top => MAX_POSITION,
            HpdDirection::Bottom => MIN_POSITION,
        }
    }

    fn get_dir_seg(&self) -> f32 {
        match self {
            HpdDirection::_Top => 1.0,
            HpdDirection::Bottom => -1.0,
        }
    }
}

#[derive(Debug)]
pub struct LinearScale {
    last_position: i32,
    last_ts: AtomiInstant,
    position: AtomicI32,
    home_position: Option<i32>,
}

impl LinearScale {
    pub fn new() -> Self {
        Self {
            last_position: 0,
            last_ts: AtomiInstant::from_ticks(0),
            position: AtomicI32::new(0),
            home_position: None,
        }
    }

    fn set_home(&mut self) {
        self.home_position = Some(self.position.load(Ordering::Relaxed));
    }

    fn get_rel_position(&self) -> Result<i32, AtomiError> {
        Ok(self.position.load(Ordering::Relaxed))
    }

    fn get_abs_position(&self) -> Result<i32, AtomiError> {
        match self.home_position {
            None => Err(AtomiError::HpdNotHomed),
            Some(h) => Ok(self.position.load(Ordering::Relaxed) - h),
        }
    }

    pub fn update_position(&mut self, new_position: i32) {
        let t = self.position.load(Ordering::Relaxed);
        if t != self.last_position {
            self.last_position = t;
            self.last_ts = now();
        }
        self.position.store(new_position, Ordering::Relaxed);
    }

    fn check_homed(&self) -> Result<i32, AtomiError> {
        self.home_position.ok_or(AtomiError::HpdNeedToHome)
    }

    fn get_distance(&self, target_position: i32) -> Result<i32, AtomiError> {
        Ok(target_position - self.get_abs_position()?)
    }

    fn is_stationary_internal(
        &self,
        time_threshold: AtomiDuration,
        distance_threshold: i32,
    ) -> bool {
        let current_position = self.position.load(Ordering::Relaxed);
        // info!(
        //     "now = {}, last_ts = {}, current_pos = {}, last_pos = {}",
        //     Debug2Format(&now()),
        //     Debug2Format(&self.last_ts),
        //     current_position,
        //     self.last_position
        // );
        if (current_position - self.last_position).abs() <= distance_threshold {
            // info!("xfguo: is_stationary_internal 2");
            if let Some(elapsed_time) = now().checked_duration_since(self.last_ts) {
                // info!("xfguo: is_stationary_internal 3");
                if elapsed_time > time_threshold {
                    info!("STATIONED: {}", current_position);
                    return true;
                }
            }
        }
        false
    }

    fn is_stationary(&self) -> bool {
        self.is_stationary_internal(STATIONARY_TIME_THRESHOLD, STATIONARY_POSITION_THRESHOLD)
    }

    fn reset_stationary(&mut self) {
        self.last_ts = now();
    }
}

pub struct PwmMotor {
    enable_pin: Pin<Gpio18, FunctionSio<SioOutput>, PullDown>,
    pwm: Slice<Pwm0, FreeRunning>,
}

impl PwmMotor {
    pub fn new(
        enable_pin: Pin<Gpio18, FunctionSio<SioOutput>, PullDown>,
        pwm: Slice<Pwm0, FreeRunning>,
    ) -> Self {
        Self { enable_pin, pwm }
    }

    fn start_pwm_motor(&mut self) -> Result<(), AtomiError> {
        self.enable_pin
            .set_high()
            .or(Err(AtomiError::HpdCannotStart))
    }

    fn apply_speed(&mut self, speed: f32) {
        let t = (-speed * 33.0) as i32;
        let duty = match t {
            1..=33 => (33 - t) as u32,
            -33..=-1 => (67 - t) as u32,
            _ => 50,
        };
        let duty_scaled = (MOTOR150_PWM_TOP as u32) * duty / 100;
        // info!("speed = {}, t = {}, duty = {}, duty_scaled = {}", speed, t, duty, duty_scaled);
        self.pwm.channel_a.set_duty(duty_scaled as u16);
        self.pwm.channel_b.set_duty(duty_scaled as u16);
    }
}

pub struct HpdProcessor {
    linear_scale: &'static mut LinearScale,
    pwm_motor: PwmMotor,
}

impl HpdProcessor {
    pub fn new(linear_scale: &'static mut LinearScale, pwm_motor: PwmMotor) -> Self {
        Self {
            linear_scale,
            pwm_motor,
        }
    }

    fn check_homed(&self) -> Result<i32, AtomiError> {
        self.linear_scale.check_homed()
    }

    pub async fn home(&mut self) -> Result<i32, AtomiError> {
        self.pwm_motor.start_pwm_motor()?;

        let dir = HpdDirection::Bottom;
        info!("xfguo: Home to one direction, dir: {}, from: {}, to: {}",
            dir, self.linear_scale.get_rel_position(), dir.get_most_position());
        self.home_on_direction(dir).await?;
        // info!(
        //     "xfguo: Home before set_top_position, current = {}",
        //     Debug2Format(self.linear_scale)
        // );
        // self.linear_scale.set_top_position();
        // dir = dir.reverse();
        // info!(
        //     "xfguo: Home to another direction, current = {}, dir: {}, target: {},",
        //     Debug2Format(self.linear_scale),
        //     dir,
        //     dir.get_most_position()
        // );
        // self.home_on_direction(dir).await?;
        info!(
            "xfguo: After homed, linear_scale = {}",
            Debug2Format(self.linear_scale)
        );
        self.linear_scale.set_home();
        info!(
            "xfguo: After homed 2, linear_scale = {}",
            Debug2Format(self.linear_scale)
        );

        self.linear_scale.get_abs_position()
    }

    async fn home_on_direction(&mut self, dir: HpdDirection) -> Result<i32, AtomiError> {
        info!("home_on_direction 0, now = {}", now().ticks());
        self.move_with_speed(dir.get_dir_seg() * 1.0, 10_000_000).await?;
        info!("home_on_direction 2, now = {}", now().ticks());
        self.move_with_speed(dir.get_dir_seg() * -1.0, 1000).await?;
        info!("home_on_direction 4, now = {}", now().ticks());
        let t = self.move_with_speed(dir.get_dir_seg() * 0.5, 10_000_000).await;
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
            // info!("pos = {}", self.linear_scale.get_rel_position());
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
        let mut pid = PIDController::new(0.1, 0.0, 0.001, target);
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
            // info!("Current pos: {}, speed = {}", pos, speed);
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
