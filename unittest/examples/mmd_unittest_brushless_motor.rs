#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use pizzaro::{
    blinky_led,
    bsp::{
        board_mmd_release_sb::{MmdBrushlessMotor0Channel, MmdBrushlessMotor1Channel},
        config::MMD_BRUSHLESS_MOTOR_PWM_TOP,
    },
    common::{brushless_motor::BrushlessMotor, led_controller::MyLED, ws2812_bitbang::Ws2812},
    mmd_bl1_ctl_channel, mmd_bl1_ctl_pwm_slice, mmd_bl2_ctl_channel, mmd_bl2_ctl_pwm_slice,
    mmd_dir_bl0, mmd_dir_bl1, mmd_spd_ctrl_bl0, mmd_spd_ctrl_bl1,
};
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry,
    gpio::Pins,
    pac,
    pwm::Slices,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let led_pin = blinky_led!(pins).into_function().into_dyn_pin();
    let mut myled = MyLED::new((10, 10, 10).into(), (0, 0, 0).into(), Ws2812::new(led_pin));

    // Init PWMs
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    let mut pwm0 = mmd_bl1_ctl_pwm_slice!(pwm_slices);
    pwm0.set_ph_correct();
    pwm0.set_top(MMD_BRUSHLESS_MOTOR_PWM_TOP);
    pwm0.enable();
    mmd_bl1_ctl_channel!(pwm0).output_to(mmd_spd_ctrl_bl0!(pins));

    let mut pwm1 = mmd_bl2_ctl_pwm_slice!(pwm_slices);
    pwm1.set_ph_correct();
    pwm1.set_top(MMD_BRUSHLESS_MOTOR_PWM_TOP);
    pwm1.enable();
    mmd_bl2_ctl_channel!(pwm1).output_to(mmd_spd_ctrl_bl1!(pins));

    let mut bl0 = BrushlessMotor::new(
        mmd_dir_bl0!(pins).into_push_pull_output().into_dyn_pin(),
        pwm0,
        MmdBrushlessMotor0Channel,
        (0.01, 0.35, 0.65, 0.99),
        false,
        MMD_BRUSHLESS_MOTOR_PWM_TOP,
    );
    let mut bl1 = BrushlessMotor::new(
        mmd_dir_bl1!(pins).into_push_pull_output().into_dyn_pin(),
        pwm1,
        MmdBrushlessMotor1Channel,
        (0.01, 0.35, 0.65, 0.97),
        false,
        MMD_BRUSHLESS_MOTOR_PWM_TOP,
    );

    let mut increasing = false;
    let mut speed: i32 = 0;
    let step = 4;
    loop {
        info!("speed: {} %", speed);
        //br.apply_speed(speed as f32 / 100.0);
        bl0.apply_speed(speed as f32 / 100.0);
        bl1.apply_speed(speed as f32 / 100.0);
        myled.ledon();
        delay.delay_ms(200);

        myled.ledoff();
        delay.delay_ms(200);
        // 调整 speed 值
        if increasing {
            speed += step;
            if speed >= 95 {
                increasing = false;
            }
        } else {
            speed -= step;
            if speed <= -95 {
                increasing = true;
            }
        }
    }
}
