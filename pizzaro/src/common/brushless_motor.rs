use cortex_m::prelude::_embedded_hal_PwmPin;
use defmt::debug;
use embedded_hal::digital::OutputPin;
use generic::atomi_error::AtomiError;
use rp2040_hal::gpio::{DynPinId, FunctionSio, Pin, PullDown, SioOutput};
use rp2040_hal::pwm::{FreeRunning, Slice, SliceId};

use super::pwm_stepper::PwmChannels;

//pub const MMD_PWM_TOP: u16 = 5000;

pub struct BrushlessMotor<S: SliceId> {
    dir_pin: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
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

impl<S: SliceId> BrushlessMotor<S> {
    pub fn new(
        dir_pin: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
        pwm: Slice<S, FreeRunning>,
        pwm_channel: PwmChannels,
        thres_speed: (f32, f32, f32, f32),
        revert_dir: bool,
        pwm_top: u16,
    ) -> Self {
        let mut motor = Self { dir_pin, pwm, pwm_channel, thres_speed, revert_dir, pwm_top };
        motor.stop().expect("Failed to stop motor");
        motor
    }

    pub(crate) fn enable(&mut self) -> Result<(), AtomiError> {
        self.pwm.enable();
        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), AtomiError> {
        self.apply_speed(0.0);
        Ok(())
    }

    pub fn apply_speed(&mut self, speed: f32) {
        // speed范围-1.0~1.0
        // 这里duty需要注意，静止为0.5，但最小最大值不能到0-1.0，必须限制在大致0.03~0.97的范围内。
        // 此外，因为阻力的缘故，启动速度也要加一个偏置，比如0.54才开始转。
        // 此外还需要注意，正向偏置和负向偏置还未必一致，所以最终实际的速度按照占空比来算大致是（0.03~0.45, 0.55~0.97这样的

        self.enable().expect("Failed to enable motor");

        let (s1, s2, s3, s4) = self.thres_speed;
        let spd = if self.revert_dir { -speed.clamp(-1.0, 1.0) } else { speed.clamp(-1.0, 1.0) };

        let spd_mapped = if spd > 0.0 {
            spd * (s4 - s3) * 2.0 // 参考s3 ~ s4，但放大到0~1区间
        } else if spd < 0.0 {
            (-spd) * (s2 - s1) * 2.0 // 参考s1 ~ s2，但放大到0~1区间，注意此时输入的spd为负
        } else {
            0.0
        };

        //let t = ((if self.revert_dir { -speed } else { speed }) * 33.0) as i32;
        let duty_scaled = ((self.pwm_top as f32) * spd_mapped) as u32;
        debug!(
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

        //self.pwm.channel_a.set_duty(duty_scaled as u16);
        //self.pwm.channel_b.set_duty(duty_scaled as u16);

        // update dir
        if spd < 0.0 {
            self.dir_pin.set_low().unwrap();
        } else {
            self.dir_pin.set_high().unwrap();
        }
    }
}
