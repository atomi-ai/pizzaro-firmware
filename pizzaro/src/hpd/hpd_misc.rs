use crate::common::global_timer::{now, AtomiDuration, AtomiInstant};
use core::sync::atomic::{AtomicI32, Ordering};
use defmt::{info, Format};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use generic::atomi_error::AtomiError;
use rp2040_hal::gpio::{DynPinId, FunctionSio, Pin, PullDown, SioOutput};
use rp2040_hal::pwm::{FreeRunning, Pwm0, Slice};

pub const MOTOR150_PWM_TOP: u16 = 2000;
// const LITTLE_DISTANCE: i32 = 1000;  // 10mm

const MAX_POSITION: i32 = 10_000_000;
const MIN_POSITION: i32 = -MAX_POSITION;
const STATIONARY_POSITION_THRESHOLD: i32 = 10;
const STATIONARY_TIME_THRESHOLD: AtomiDuration = AtomiDuration::millis(200);

#[derive(Copy, Clone, Debug, Format)]
pub enum HpdDirection {
    _Top, // Add "Top" back if necessary.
    Bottom,
}

impl HpdDirection {
    pub(crate) fn get_most_position(&self) -> i32 {
        match self {
            HpdDirection::_Top => MAX_POSITION,
            HpdDirection::Bottom => MIN_POSITION,
        }
    }

    pub(crate) fn get_dir_seg(&self) -> f32 {
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

impl Default for LinearScale {
    fn default() -> Self {
        Self::new()
    }
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

    pub(crate) fn set_home(&mut self) {
        self.home_position = Some(self.position.load(Ordering::Relaxed));
    }

    pub(crate) fn get_rel_position(&self) -> Result<i32, AtomiError> {
        Ok(self.position.load(Ordering::Relaxed))
    }

    pub(crate) fn get_abs_position(&self) -> Result<i32, AtomiError> {
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

    pub(crate) fn check_homed(&self) -> Result<i32, AtomiError> {
        self.home_position.ok_or(AtomiError::HpdNeedToHome)
    }

    pub(crate) fn get_distance(&self, target_position: i32) -> Result<i32, AtomiError> {
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

    pub(crate) fn is_stationary(&self) -> bool {
        self.is_stationary_internal(STATIONARY_TIME_THRESHOLD, STATIONARY_POSITION_THRESHOLD)
    }

    pub(crate) fn reset_stationary(&mut self) {
        self.last_ts = now();
    }
}

pub struct PwmMotor {
    enable_pin: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
    pwm: Slice<Pwm0, FreeRunning>,
}

impl PwmMotor {
    pub fn new(
        enable_pin: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
        pwm: Slice<Pwm0, FreeRunning>,
    ) -> Self {
        Self { enable_pin, pwm }
    }

    pub(crate) fn start_pwm_motor(&mut self) -> Result<(), AtomiError> {
        self.enable_pin
            .set_high()
            .or(Err(AtomiError::HpdCannotStart))
    }

    pub(crate) fn apply_speed(&mut self, speed: f32) {
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
