#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use pizzaro::{
    blinky_led,
    bsp::board_mmd_release_sb::{MmdMotor42Step1Channel, MmdMotor57StepChannel},
    common::{led_controller::MyLED, pwm_stepper::PwmStepper, ws2812_bitbang::Ws2812},
    mmd_motor42_pwm_slice0, mmd_motor42_pwm_slice1, mmd_motor42_step0_channel,
    mmd_motor42_step1_channel, mmd_stepper42_dir0, mmd_stepper42_dir1, mmd_stepper42_nEN0,
    mmd_stepper42_nEN1, mmd_stepper42_step0, mmd_stepper42_step1, mmd_stepper57_dir,
    mmd_stepper57_nEN, mmd_stepper57_pwm_slice, mmd_stepper57_step, mmd_stepper57_step_channel,
    mmd_tmc_uart_tx,
};

use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry,
    gpio::{DynPinId, FunctionSio, Pin, Pins, PullDown, SioOutput},
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

    // 第一个42步进
    let enable_pin_42_0 = Some(mmd_stepper42_nEN0!(pins).into_push_pull_output().into_dyn_pin());
    let dir_pin_42_0 = mmd_stepper42_dir0!(pins).into_push_pull_output().into_dyn_pin();
    let mut pwm_42_0 = mmd_motor42_pwm_slice0!(pwm_slices);
    mmd_motor42_step0_channel!(pwm_42_0).output_to(mmd_stepper42_step0!(pins));

    // 使用第二个通道连接驱动传送带旋转的电机
    let enable_pin_42_1: Option<Pin<DynPinId, FunctionSio<SioOutput>, PullDown>> =
        mmd_stepper42_nEN1!(pins);
    let dir_pin_42_1 = mmd_stepper42_dir1!(pins).into_push_pull_output().into_dyn_pin();
    let mut pwm_42_1 = mmd_motor42_pwm_slice1!(pwm_slices);
    mmd_motor42_step1_channel!(pwm_42_1).output_to(mmd_stepper42_step1!(pins));

    // init 57
    let enable_pin_57 = Some(mmd_stepper57_nEN!(pins).into_push_pull_output().into_dyn_pin());
    let dir_pin_57 = mmd_stepper57_dir!(pins).into_push_pull_output().into_dyn_pin();
    let mut pwm_57 = mmd_stepper57_pwm_slice!(pwm_slices);
    mmd_stepper57_step_channel!(pwm_57).output_to(mmd_stepper57_step!(pins));

    // init tmc uart
    let tmc_uart = mmd_tmc_uart_tx!(pins).into_pull_down_disabled();

    let mut stepper42_0 = PwmStepper::new(
        enable_pin_42_0,
        dir_pin_42_0,
        MmdMotor42Step1Channel,
        pwm_42_0,
        200, // 无细分，一圈200脉冲
        false,
    );
    let mut stepper42_1 = PwmStepper::new(
        enable_pin_42_1, // enable_pin_42_1,
        dir_pin_42_1,
        MmdMotor42Step1Channel,
        pwm_42_1,
        200, // 无细分，一圈200脉冲
        false,
    );

    let mut stepper57 = PwmStepper::new(
        enable_pin_57,
        dir_pin_57,
        MmdMotor57StepChannel,
        pwm_57,
        200, // 无细分，一圈200脉冲
        false,
    );

    stepper57.stop();
    stepper42_0.stop();
    stepper42_1.stop();

    let mut increasing = false;
    let mut speed: i32 = 0;
    let step = 4;
    loop {
        info!("speed: {} %", speed);
        stepper42_0.set_speed(speed);
        stepper42_1.set_speed(speed);
        stepper57.set_speed(speed * 5);
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
