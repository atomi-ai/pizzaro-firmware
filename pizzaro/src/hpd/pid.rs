use defmt::{debug, info};

use crate::common::global_timer::{AtomiDuration, AtomiInstant};

const UNSET_INSTANT: AtomiInstant = AtomiInstant::from_ticks(0);
const STATIONARY_POSITION_THRESHOLD: f32 = 10.0;
const STATIONARY_TIME_THRESHOLD: AtomiDuration = AtomiDuration::millis(200);

#[derive(Debug)]
pub struct PIDController {
    kp: f32,
    ki: f32,
    kd: f32,
    setpoint: f32,
    last_error: f32,
    integral: f32,
    last_pos: i32,
    last_speed: f32,
    last_ts_err_in_range: AtomiInstant,
}

impl defmt::Format for PIDController {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{:?}", defmt::Debug2Format(self));
    }
}

impl PIDController {
    pub fn new(kp: f32, ki: f32, kd: f32, setpoint: i32) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            setpoint: setpoint as f32,
            integral: 0.0,
            last_error: 0.0,
            last_pos: -10_000_000,
            last_speed: -1.0,
            last_ts_err_in_range: UNSET_INSTANT,
        }
    }

    pub fn reach_target(&mut self, current_position: i32, ts: AtomiInstant) -> bool {
        let error: f32 = self.setpoint - current_position as f32;
        let absolute_error = if error < 0.0 { -error } else { error };

        debug!(
            "reach_target?, error:{}, abs_error:{}, cur_pos:{}, setpoint:{}",
            error, absolute_error, current_position, self.setpoint
        );

        if absolute_error > STATIONARY_POSITION_THRESHOLD {
            self.last_ts_err_in_range = UNSET_INSTANT;
            return false;
        }
        if self.last_ts_err_in_range == UNSET_INSTANT {
            self.last_ts_err_in_range = ts;
        }
        if let Some(elapsed_time) = ts.checked_duration_since(self.last_ts_err_in_range) {
            if elapsed_time > STATIONARY_TIME_THRESHOLD {
                info!("REACHED TARGET: pos: {}, ts: {}", current_position, ts.ticks());
                return true;
            }
        }
        false
    }

    pub fn calculate(&mut self, current_position: i32, dt: f32) -> f32 {
        if self.last_pos == current_position {
            // keep previous strategy, don't aggregate the error.
            return self.last_speed;
        }
        let error = self.setpoint - current_position as f32;
        self.integral += error * dt;
        let derivative = (error - self.last_error) / dt;
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        debug!(
            "PID: current_position: {}, error: {}, der: {}, self: {}, output: {}",
            current_position, error, derivative, self, output
        );

        self.last_error = error;
        self.last_pos = current_position;
        self.last_speed = output.clamp(-10.0, 10.0) * 0.1;

        self.last_speed
    }
}
