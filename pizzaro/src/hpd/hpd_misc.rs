use crate::bsp::config::REVERT_HPD_LINEARSCALE_DIRECTION;
use crate::common::global_timer::{now, AtomiDuration, AtomiInstant};
use core::sync::atomic::{AtomicI32, Ordering};
use defmt::{info, Format};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use generic::atomi_error::AtomiError;
use rp2040_hal::gpio::{DynPinId, FunctionSio, Pin, PullDown, SioOutput};
use rp2040_hal::pac::adc::fcs::THRESH_R;
use rp2040_hal::pwm::{FreeRunning, Pwm0, Slice};

pub const MOTOR150_PWM_TOP: u16 = 5000;
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
    revert_linearscale_dir: bool,
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
            revert_linearscale_dir: REVERT_HPD_LINEARSCALE_DIRECTION,
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
        self.position.store(
            if self.revert_linearscale_dir {
                -new_position
            } else {
                new_position
            },
            Ordering::Relaxed,
        );
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
    thres_speed: (f32, f32, f32, f32),
    revert_dir: bool,
    revert_en: bool,
}

impl PwmMotor {
    pub fn new(
        enable_pin: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
        pwm: Slice<Pwm0, FreeRunning>,
        thres_speed: (f32, f32, f32, f32),
        revert_dir: bool,
        revert_en: bool,
    ) -> Self {
        Self {
            enable_pin,
            pwm,
            thres_speed,
            revert_dir,
            revert_en,
        }
    }

    pub(crate) fn start_pwm_motor(&mut self) -> Result<(), AtomiError> {
        (if self.revert_en {
            self.enable_pin.set_low()
        } else {
            self.enable_pin.set_high()
        })
        .or(Err(AtomiError::HpdCannotStart))
    }

    pub(crate) fn apply_speed(&mut self, speed: f32) {
        // speed范围-1.0~1.0
        // 这里duty需要注意，静止为0.5，但最小最大值不能到0-1.0，必须限制在大致0.03~0.97的范围内。
        // 此外，因为阻力的缘故，启动速度也要加一个偏置，比如0.54才开始转。
        // 此外还需要注意，正向偏置和负向偏置还未必一致，所以最终实际的速度按照占空比来算大致是（0.03~0.45, 0.55~0.97这样的

        let (s1, s2, s3, s4) = self.thres_speed;
        let spd = if self.revert_dir {
            -speed.clamp(-1.0, 1.0)
        } else {
            speed.clamp(-1.0, 1.0)
        };

        let spd_mapped = if spd > 0.0 {
            spd * (s4 - s3) + s3
        } else if spd < 0.0 {
            (spd + 1.0) * (s2 - s1) + s1
        } else {
            0.5
        };

        //let t = ((if self.revert_dir { -speed } else { speed }) * 33.0) as i32;
        let duty_scaled = ((MOTOR150_PWM_TOP as f32) * spd_mapped) as u32;
        // info!(
        //     "speed = {}, spd_mapped = {}, duty_scaled = {}, revert_dir={}",
        //     speed, spd_mapped, duty_scaled, self.revert_dir,
        // );
        self.pwm.channel_a.set_duty(duty_scaled as u16);
        self.pwm.channel_b.set_duty(duty_scaled as u16);
    }
}
