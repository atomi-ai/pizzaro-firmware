use cortex_m::prelude::_embedded_hal_PwmPin;
use defmt::{debug, info};
use embedded_hal::digital::StatefulOutputPin;
use generic::atomi_error::AtomiError;
use rp2040_hal::pwm::{FreeRunning, Slice, SliceId};

use crate::common::brush_motor::spd_mapping;

use super::pwm_stepper::PwmChannels;
const ENABLE_LOGIC_HIGH: bool = true;

// pub const MMD_PWM_TOP: u16 = 5000;

///基于淘宝上买的MD03驱动板
//#[allow(non_snake_case)]
pub struct BrushMotorPatched<S: SliceId, E: StatefulOutputPin> {
    enable_pin: E,
    dir_pin: E,
    pwm: Slice<S, FreeRunning>,
    pwm_channel: PwmChannels,
    /// 有些驱动的最大速度不能到100%，此外电机需要克服阻力才能启动，因此也需要知道它的最小启动速度
    /// 左转最大速度，左转最小速度，右转最小速度，右转最大速度
    thres_speed: (f32, f32, f32, f32),
    /// 电机正负如果接反，运转方向会相反
    revert_dir: bool,
    /// 传入配置的PWM_TOP
    pwm_top: u16,
}

impl<S: SliceId, E: StatefulOutputPin> BrushMotorPatched<S, E> {
    pub fn new(
        enable_pin: E,
        dir_pin: E,
        pwm: Slice<S, FreeRunning>,
        pwm_channel: PwmChannels,
        thres_speed: (f32, f32, f32, f32),
        revert_dir: bool,
        pwm_top: u16,
    ) -> Self {
        let mut motor =
            Self { enable_pin, dir_pin, pwm, pwm_channel, thres_speed, revert_dir, pwm_top };
        motor.disable().expect("Failed to disable motor");
        motor
    }

    pub(crate) fn enable(&mut self) -> Result<(), AtomiError> {
        if ENABLE_LOGIC_HIGH {
            self.enable_pin.set_high().map_err(|_| AtomiError::GpioPinError)?;
        } else {
            self.enable_pin.set_low().map_err(|_| AtomiError::GpioPinError)?;
        }

        info!("enable motor pwm");
        self.pwm.enable();

        Ok(())
    }

    pub(crate) fn disable(&mut self) -> Result<(), AtomiError> {
        debug!("disable motor, set enable_pin => LOW");
        if ENABLE_LOGIC_HIGH {
            self.enable_pin.set_low().map_err(|_| AtomiError::GpioPinError)?;
        } else {
            self.enable_pin.set_high().map_err(|_| AtomiError::GpioPinError)?;
        }

        debug!("disable motor pwm");
        self.pwm.disable();
        Ok(())
    }

    pub fn ensure_enable(&mut self) -> Result<(), AtomiError> {
        if ENABLE_LOGIC_HIGH
            && self.enable_pin.is_set_low().map_err(|_| AtomiError::GpioPinError)?
            || !ENABLE_LOGIC_HIGH
                && self.enable_pin.is_set_high().map_err(|_| AtomiError::GpioPinError)?
        {
            self.enable()
        } else {
            Ok(())
        }
    }

    pub fn apply_speed(&mut self, speed: f32) {
        // speed范围-1.0~1.0
        // 这里duty需要注意，静止为0.5，但最小最大值不能到0-1.0，必须限制在大致0.03~0.97的范围内。
        // 此外，因为阻力的缘故，启动速度也要加一个偏置，比如0.54才开始转。
        // 此外还需要注意，正向偏置和负向偏置还未必一致，所以最终实际的速度按照占空比来算大致是（0.03~0.45, 0.55~0.97这样的

        let spd = if self.revert_dir { -speed.clamp(-1.0, 1.0) } else { speed.clamp(-1.0, 1.0) };

        let spd_mapped = spd_mapping(spd, self.thres_speed);
        info!("spd:{}, spd_mapped:{}", spd, spd_mapped);

        if spd_mapped == 0.0 {
            self.disable().expect("Failed to disable motor");
        } else {
            self.ensure_enable().expect("Failed to enable motor");
        }

        //let t = ((if self.revert_dir { -speed } else { speed }) * 33.0) as i32;
        let duty_scaled = ((self.pwm_top as f32) * spd_mapped) as u32;
        info!(
            "speed = {}, spd_mapped = {}, duty_scaled = {}, revert_dir={}",
            speed, spd_mapped, duty_scaled, self.revert_dir,
        );
        match self.pwm_channel {
            PwmChannels::channel_a => {
                self.pwm.channel_a.set_duty(duty_scaled as u16);
            }
            PwmChannels::channel_b => {
                self.pwm.channel_b.set_duty(duty_scaled as u16);
            }
        }

        // update dir
        if spd < 0.0 {
            self.dir_pin.set_low().map_err(|_| AtomiError::GpioPinError).unwrap();
        } else {
            self.dir_pin.set_high().map_err(|_| AtomiError::GpioPinError).unwrap();
        }
    }
}
