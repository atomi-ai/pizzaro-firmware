use defmt::{info, warn};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use fugit::HertzU32;
use generic::atomi_error::AtomiError;
use rp2040_hal::gpio::{DynPinId, FunctionSio, Pin, PullDown, SioOutput};
use rp2040_hal::pwm::{FreeRunning, Slice, SliceId};

// pub const MMD_PWM_TOP: u16 = 5000;

#[allow(non_camel_case_types)]
pub enum PwmChannels {
    channel_a,
    channel_b,
}

pub struct PwmStepper<S: SliceId> {
    enable_pin: Option<Pin<DynPinId, FunctionSio<SioOutput>, PullDown>>,
    dir_pin: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
    step_channel: PwmChannels,
    sys_clock: HertzU32,
    pwm: Slice<S, FreeRunning>,
    /// 电机正负如果接反，运转方向会相反
    pulse_per_round: u32,
    revert_dir: bool,
}

impl<S: SliceId> PwmStepper<S> {
    pub fn new(
        enable_pin: Option<Pin<DynPinId, FunctionSio<SioOutput>, PullDown>>,
        dir_pin: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
        step_channel: PwmChannels,
        sys_clock: HertzU32,
        pwm: Slice<S, FreeRunning>,
        pulse_per_round: u32,
        revert_dir: bool,
    ) -> Self {
        Self {
            enable_pin,
            dir_pin,
            step_channel,
            sys_clock,
            pwm,
            pulse_per_round,
            revert_dir,
        }
    }

    pub(crate) fn enable(&mut self) -> Result<(), AtomiError> {
        if let Some(ref mut en) = self.enable_pin {
            en.set_low().or(Err(AtomiError::MmdCannotStart))
        } else {
            Ok(())
        }
    }

    fn set_pwm_freq(&mut self, freq: u32) -> u16 {
        const TOP_MAX: u32 = 65534;
        let source_hz = self.sys_clock.to_Hz();
        let mut div16: u32;
        let mut top: u32;

        if (source_hz + freq / 2) / freq < TOP_MAX {
            div16 = 16;
            top = (source_hz + freq / 2) / freq - 1;
        } else {
            div16 = get_slice_hz_ceil(self.sys_clock, TOP_MAX * freq);
            top = get_slice_hz_round(self.sys_clock, div16 * freq) - 1;
        }

        if div16 < 16 {
            warn!("Requested frequency too large, clamping to maximum");
            div16 = 16;
            top = TOP_MAX;
        } else if div16 >= 256 * 16 {
            warn!("Requested frequency too small, clamping to minimum");
            div16 = 256 * 16 - 1;
            top = get_slice_hz_round(self.sys_clock, div16 * freq) - 1;
        }

        let div_int = (div16 / 16) as u8;
        let div_frac = ((div16 % 16) * 16) as u8;
        self.pwm.set_div_int(div_int);
        self.pwm.set_div_frac(div_frac);
        self.pwm.set_top(top as u16);
        self.pwm.set_counter(0);
        self.pwm.enable();
        top as u16
    }

    pub(crate) fn stop(&mut self) {
        self.pwm.disable();
    }

    pub(crate) fn set_speed(&mut self, speed_rpm: i32) {
        info!("set speed:{}", speed_rpm);
        if speed_rpm == 0 {
            self.stop();
            return;
        }
        let spd = if self.revert_dir {
            -speed_rpm
        } else {
            speed_rpm
        };

        // spd: rpm
        // spd / 60 -> round per second
        // spd / 60 * pulse_per_round -> pulse per second
        // pulse width -> 1s / (spd * 60 * pulse_per_round)
        // freq -> 1/ pulse width -> (spd / 60 * pulse_per_round)/1

        let freq = speed_rpm.unsigned_abs() * self.pulse_per_round / 60;
        let freq_top = self.set_pwm_freq(freq);
        match self.step_channel {
            PwmChannels::channel_a => {
                self.pwm.channel_a.set_duty(freq_top / 2);
            }
            PwmChannels::channel_b => {
                self.pwm.channel_b.set_duty(freq_top / 2);
            }
        }

        if spd < 0 {
            self.dir_pin.set_low().unwrap();
        } else {
            self.dir_pin.set_high().unwrap();
        }
    }
}

fn get_slice_hz(sys_clk: HertzU32, offset: u32, div16: u32) -> u32 {
    let source_hz = sys_clk.to_Hz();
    if source_hz + offset / 16 > 268000000 {
        ((16 * source_hz as u64 + offset as u64) / div16 as u64) as u32
    } else {
        (16 * source_hz + offset) / div16
    }
}

fn get_slice_hz_round(sys_clk: HertzU32, div16: u32) -> u32 {
    get_slice_hz(sys_clk, div16 / 2, div16)
}

fn get_slice_hz_ceil(sys_clk: HertzU32, div16: u32) -> u32 {
    get_slice_hz(sys_clk, div16 - 1, div16)
}
