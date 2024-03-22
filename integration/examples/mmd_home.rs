#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;

use cortex_m::asm::delay;
use defmt::info;
use pizzaro::common::led_controller::{blinky_smart_led, MyLED};
use pizzaro::{
    mmd_limit0, mmd_limit1, mmd_stepper42_dir0, mmd_stepper42_nEN0, mmd_stepper42_step0, smart_led,
};
use rp2040_hal::Clock;
use rp2040_hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog, Timer};
use rp_pico::{entry, XOSC_CRYSTAL_FREQ};

use pizzaro::common::async_initialization;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, DelayCreator};
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::mmd::linear_stepper::LinearStepper;
use pizzaro::mmd::mmd_dispatcher::LinearStepperType;
use pizzaro::mmd::stepper::Stepper;
use rp2040_hal::pio::PIOExt;
use ws2812_pio::Ws2812Direct;

#[entry]
fn main() -> ! {
    async_initialization();
    let mut pac = pac::Peripherals::take().unwrap();
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let sio = Sio::new(pac.SIO);
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    init_global_timer(Box::new(Rp2040Timer::new(timer)));

    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    {
        let enable_pin = mmd_stepper42_nEN0!(pins).into_push_pull_output();
        let dir_pin = mmd_stepper42_dir0!(pins).into_push_pull_output();
        let step_pin = mmd_stepper42_step0!(pins).into_push_pull_output();
        let left_limit_pin = mmd_limit0!(pins).into_pull_down_input();
        let right_limit_pin = mmd_limit1!(pins).into_pull_down_input();
        let delay_creator = DelayCreator::new();
        let linear_stepper = LinearStepper::new(
            Stepper::new(enable_pin, dir_pin, step_pin, delay_creator),
            left_limit_pin,
            right_limit_pin,
        );
        spawn_task(mmd_home(linear_stepper));
    }

    {
        // let mut led = pins.gpio2.into_push_pull_output().into_dyn_pin();
        // spawn_task(blinky(led));

        let my_led = MyLED::new(
            (1, 1, 1).into(),
            (0, 0, 0).into(),
            Ws2812Direct::new(
                smart_led!(pins).into_function().into_dyn_pin(),
                &mut pio,
                sm0,
                clocks.peripheral_clock.freq(),
            ),
        );

        spawn_task(blinky_smart_led(my_led));
    }

    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

async fn mmd_home(mut linear_stepper: LinearStepperType) {
    let t = linear_stepper.move_to_relative(100).await;
    info!("Result of move before homing: {}", t);

    let t = linear_stepper.home().await;
    info!("Home done, t = {}", t);

    let _t = linear_stepper.move_to_relative(100).await;
    let _t = linear_stepper.move_to_relative(100).await;
    let _t = linear_stepper.move_to_relative(-100).await;
    let _t = linear_stepper.move_to_relative(100).await;
    let _t = linear_stepper.move_to(100).await;
    let _t = linear_stepper.move_to(500).await;
}
