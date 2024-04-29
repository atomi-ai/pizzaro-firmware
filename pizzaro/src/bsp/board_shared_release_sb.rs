/// Shared definitions
///
/// TODO(lv): Should we use "rp_pico::" directly?
use crate::define_pins;

define_pins! {
    // ws2812b led on mc/mmd/hpd
    smart_led, gpio16,

    demo_pwm_a, gpio0,
    demo_pwm_b, gpio1,
    demo_pwm_en, gpio2,
    demo_ols_a, gpio10,
    demo_ols_b, gpio11
}

define_pins! {
    demo_pwm_slice, pwm0,
    demo_pwm_channel_a, channel_a,
    demo_pwm_channel_b, channel_b
}
