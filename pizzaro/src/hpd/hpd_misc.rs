use crate::bsp::config::REVERT_HPD_LINEARSCALE_DIRECTION;
use crate::common::global_timer::{now, AtomiDuration, AtomiInstant};
use crate::hpd::stationary_detector::StationaryDetector;
use core::sync::atomic::{AtomicI32, Ordering};
use defmt::{info, Format};
use generic::atomi_error::AtomiError;

// pub const MOTOR150_PWM_TOP: u16 = 5000;
// const LITTLE_DISTANCE: i32 = 1000;  // 10mm

const MAX_POSITION: i32 = 10_000_000;
const MIN_POSITION: i32 = -MAX_POSITION;
const STATIONARY_POSITION_THRESHOLD: i32 = 10;
const STATIONARY_TIME_THRESHOLD: AtomiDuration = AtomiDuration::millis(1000);

#[derive(Copy, Clone, Debug, Format)]
pub enum LinearBullDirection {
    _Top, // Add "Top" back if necessary.
    Bottom,
}

impl LinearBullDirection {
    pub(crate) fn get_most_position(&self) -> i32 {
        match self {
            LinearBullDirection::_Top => MAX_POSITION,
            LinearBullDirection::Bottom => MIN_POSITION,
        }
    }

    pub(crate) fn get_dir_seg(&self) -> f32 {
        match self {
            LinearBullDirection::_Top => 1.0,
            LinearBullDirection::Bottom => -1.0,
        }
    }
}

const WINDOW_SIZE: usize = 8;

#[derive(Debug)]
pub struct LinearScale {
    last_position: i32,
    last_ts: AtomiInstant,
    position: AtomicI32,
    home_position: Option<i32>,
    revert_linearscale_dir: bool,
    stationary_detector: StationaryDetector<WINDOW_SIZE>,
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
            stationary_detector: StationaryDetector::new(10_000, 20.0),
        }
    }

    pub(crate) fn set_home(&mut self) {
        self.home_position = Some(self.position.load(Ordering::Relaxed));
    }

    pub fn get_rel_position(&self) -> Result<i32, AtomiError> {
        Ok(self.position.load(Ordering::Relaxed))
    }

    pub(crate) fn get_abs_position(&self) -> Result<i32, AtomiError> {
        match self.home_position {
            None => Err(AtomiError::HpdNotHomed),
            Some(h) => Ok(self.position.load(Ordering::Relaxed) - h),
        }
    }

    pub fn update_position(&mut self, new_position: i32) {
        let to_store_pos = if self.revert_linearscale_dir { -new_position } else { new_position };

        let t = self.position.load(Ordering::Relaxed);
        self.position.store(to_store_pos, Ordering::Relaxed);

        if t != self.last_position {
            self.last_position = t;
            self.last_ts = now();
        }
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

    pub(crate) fn is_stationary(&mut self) -> bool {
        self.is_stationary_internal(STATIONARY_TIME_THRESHOLD, STATIONARY_POSITION_THRESHOLD)
    }

    pub(crate) fn is_stationary_2(&mut self) -> bool {
        let ts = now().ticks() as i64;
        let pos = self.position.load(Ordering::Relaxed) as f64;
        self.stationary_detector.is_stationary(ts, pos)
    }

    pub(crate) fn reset_stationary(&mut self) {
        self.last_ts = now();
        self.stationary_detector.reset_stationary();
    }
}
