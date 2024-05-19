#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use pizzaro::bsp::board_mmd_release_sb::MmdBrushMotorChannel;
use pizzaro::{
    blinky_led,
    bsp::config::MMD_PERISTALTIC_PUMP_PWM_TOP,
    common::{brush_motor_patch::BrushMotorPatched, led_controller::MyLED, ws2812_bitbang::Ws2812},
    mmd_br0_pwm_slice, mmd_br_channel_a, mmd_br_nEN, mmd_br_pwm_a, mmd_br_pwm_b,
};
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry,
    gpio::Pins,
    pac, pwm,
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
    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let mut pwm = mmd_br0_pwm_slice!(pwm_slices);
    pwm.set_ph_correct();
    pwm.set_top(MMD_PERISTALTIC_PUMP_PWM_TOP);
    pwm.enable();
    mmd_br_channel_a!(pwm).output_to(mmd_br_pwm_a!(pins));

    let mut br = BrushMotorPatched::new(
        mmd_br_nEN!(pins).into_push_pull_output().into_dyn_pin(),
        mmd_br_pwm_b!(pins).into_push_pull_output().into_dyn_pin(),
        pwm,
        MmdBrushMotorChannel,
        (0.03, 0.45, 0.55, 0.97),
        false,
        MMD_PERISTALTIC_PUMP_PWM_TOP,
    );

    let mut increasing = true;
    let mut speed: i32 = 0;
    let step = 4;
    loop {
        info!("speed: {} %", speed);
        br.apply_speed(speed as f32 / 100.0);
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
