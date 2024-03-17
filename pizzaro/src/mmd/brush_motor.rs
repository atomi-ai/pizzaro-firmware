use defmt::info;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use generic::atomi_error::AtomiError;
use rp2040_hal::gpio::{DynPinId, FunctionSio, Pin, PullDown, SioOutput};
use rp2040_hal::pwm::{FreeRunning, Slice, SliceId};

pub const MMD_PWM_TOP: u16 = 5000;

pub struct BrushMotor<S: SliceId> {
    enable_pin: Option<Pin<DynPinId, FunctionSio<SioOutput>, PullDown>>,
    pwm: Slice<S, FreeRunning>,
    /// 有些驱动的最大速度不能到100%，此外电机需要克服阻力才能启动，因此也需要知道它的最小启动速度
    /// 左转最大速度，左转最小速度，右转最小速度，右转最大速度
    thres_speed: (f32, f32, f32, f32),
    /// 电机正负如果接反，运转方向会相反
    revert_dir: bool,
    /// 有些电机驱动器的en逻辑是反的
    is_nEN: bool,
}

impl<S: SliceId> BrushMotor<S> {
    pub fn new(
        enable_pin: Option<Pin<DynPinId, FunctionSio<SioOutput>, PullDown>>,
        pwm: Slice<S, FreeRunning>,
        thres_speed: (f32, f32, f32, f32),
        revert_dir: bool,
        is_nEN: bool,
    ) -> Self {
        Self {
            enable_pin,
            pwm,
            thres_speed,
            revert_dir,
            is_nEN,
        }
    }

    pub(crate) fn start_pwm_motor(&mut self) -> Result<(), AtomiError> {
        if let Some(ref mut en) = self.enable_pin {
            (if self.is_nEN {
                en.set_low()
            } else {
                en.set_high()
            })
            .or(Err(AtomiError::MmdCannotStart))
        } else {
            Ok(())
        }
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
        let duty_scaled = ((MMD_PWM_TOP as f32) * spd_mapped) as u32;
        info!(
            "speed = {}, spd_mapped = {}, duty_scaled = {}, revert_dir={}",
            speed, spd_mapped, duty_scaled, self.revert_dir,
        );
        self.pwm.channel_a.set_duty(duty_scaled as u16);
        self.pwm.channel_b.set_duty(duty_scaled as u16);
    }
}
