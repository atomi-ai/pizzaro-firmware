use defmt::{debug, info};
use embedded_hal::digital::v2::StatefulOutputPin;
use embedded_hal::PwmPin;
use generic::atomi_error::AtomiError;
use rp2040_hal::pwm::{FreeRunning, Slice, SliceId};

//pub const MMD_PWM_TOP: u16 = 50000;

#[allow(non_snake_case)]
pub struct BrushMotor<S: SliceId, E: StatefulOutputPin> {
    enable_pin: E,
    pwm: Slice<S, FreeRunning>,
    /// 有些驱动的最大速度不能到100%，此外电机需要克服阻力才能启动，因此也需要知道它的最小启动速度
    /// 左转最大速度，左转最小速度，右转最小速度，右转最大速度
    thres_speed: (f32, f32, f32, f32),
    /// 电机正负如果接反，运转方向会相反
    revert_dir: bool,
    /// 有些电机驱动器的en逻辑是反的
    is_nEN: bool,
    /// 传入配置的PWM_TOP
    pwm_top: u16,
}

impl<S: SliceId, E: StatefulOutputPin> BrushMotor<S, E> {
    pub fn new(
        enable_pin: E,
        pwm: Slice<S, FreeRunning>,
        thres_speed: (f32, f32, f32, f32),
        revert_dir: bool,
        #[allow(non_snake_case)] is_nEN: bool,
        pwm_top: u16,
    ) -> Self {
        let mut motor = Self { enable_pin, pwm, thres_speed, revert_dir, is_nEN, pwm_top };
        motor.disable().expect("Failed to disable motor");
        motor
    }

    pub(crate) fn enable(&mut self) -> Result<(), AtomiError> {
        if self.is_nEN {
            self.enable_pin.set_low().map_err(|_| AtomiError::GpioPinError)?
        } else {
            self.enable_pin.set_high().map_err(|_| AtomiError::GpioPinError)?
        }
        info!("enable motor pwm");
        self.pwm.enable();

        Ok(())
    }

    pub(crate) fn disable(&mut self) -> Result<(), AtomiError> {
        if self.is_nEN {
            info!("disable motor, set enable_pin => HIGH");
            self.enable_pin.set_high().map_err(|_| AtomiError::GpioPinError)?
        } else {
            info!("disable motor, set enable_pin => LOW");
            self.enable_pin.set_low().map_err(|_| AtomiError::GpioPinError)?
        }

        debug!("disable motor pwm");
        self.pwm.disable();
        Ok(())
    }

    pub fn ensure_enable(&mut self) -> Result<(), AtomiError> {
        if self.enable_pin.is_set_low().map_err(|_| AtomiError::GpioPinError)? != self.is_nEN {
            self.enable()
        } else {
            Ok(())
        }
    }

    pub(crate) fn apply_speed(&mut self, speed: f32) {
        self.apply_speed_freerun(speed, true)
    }

    /// 设置运行速度
    /// * speed: 范围-1.0 ~ 1.0
    /// * freerun: 当速度接近0的时候是否自动切断对电机的控制。
    pub(crate) fn apply_speed_freerun(&mut self, speed: f32, freerun: bool) {
        // speed范围-1.0~1.0
        // 这里duty需要注意，静止为0.5，但最小最大值不能到0-1.0，必须限制在大致0.03~0.97的范围内。
        // 此外，因为阻力的缘故，启动速度也要加一个偏置，比如0.54才开始转。
        // 此外还需要注意，正向偏置和负向偏置还未必一致，所以最终实际的速度按照占空比来算大致是（0.03~0.45, 0.55~0.97这样的

        let (s1, s2, s3, s4) = self.thres_speed;
        let spd = if self.revert_dir { -speed.clamp(-1.0, 1.0) } else { speed.clamp(-1.0, 1.0) };

        debug!("spd:{}", spd);
        let spd_mapped = if spd > 0.0 {
            self.ensure_enable().expect("Failed to enable motor");
            spd * (s4 - s3) + s3
        } else if spd < 0.0 {
            self.ensure_enable().expect("Failed to enable motor");
            (spd + 1.0) * (s2 - s1) + s1
        } else {
            debug!("disable pwm");
            if freerun {
                self.disable().expect("Failed to disable motor");
            }
            0.5
        };

        //let t = ((if self.revert_dir { -speed } else { speed }) * 33.0) as i32;
        let duty_scaled = ((self.pwm_top as f32) * spd_mapped) as u32;
        debug!(
            "speed = {}, spd_mapped = {}, duty_scaled = {}, revert_dir={}",
            speed, spd_mapped, duty_scaled, self.revert_dir,
        );
        self.pwm.channel_a.set_duty(duty_scaled as u16);
        self.pwm.channel_b.set_duty((duty_scaled) as u16);
    }
}
