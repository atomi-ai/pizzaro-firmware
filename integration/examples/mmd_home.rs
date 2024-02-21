#![no_std]
#![no_main]

extern crate alloc;

// TODO(zephyr): Move these two default definition to a global lib.
use panic_probe as _;
use defmt_rtt as _;

use alloc::boxed::Box;
use cortex_m::asm::delay;
use defmt::{info};
use rp2040_hal::{clocks::{init_clocks_and_plls}, pac, sio::Sio, Timer, watchdog::Watchdog};
use rp_pico::{entry, XOSC_CRYSTAL_FREQ};
use common::async_initialization;
use common::executor::{spawn_task, start_global_executor};
use common::global_timer::{DelayCreator, init_global_timer};
use common::rp2040_timer::Rp2040Timer;
use integration::mmd::linear_stepper::LinearStepper;
use integration::mmd::stepper::Stepper;
use integration::mmd_processor::{MmdProcessor};

#[entry]
fn main() -> ! {
    async_initialization();
    let mut pac = pac::Peripherals::take().unwrap();
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

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    {
        let enable_pin = pins.gpio22.into_push_pull_output();
        let dir_pin = pins.gpio18.into_push_pull_output();
        let step_pin = pins.gpio11.into_push_pull_output();
        let left_limit_pin = pins.gpio26.into_pull_down_input();
        let right_limit_pin = pins.gpio27.into_pull_down_input();
        let delay_creator = DelayCreator::new();
        let processor = MmdProcessor::new(
            LinearStepper::new(
                Stepper::new(enable_pin, dir_pin, step_pin, delay_creator),
                left_limit_pin,
                right_limit_pin));
        spawn_task(mmd_home(processor));
    }

    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

async fn mmd_home(mut mmd_processor: MmdProcessor) {
    let t = mmd_processor.move_to_relative(100).await;
    info!("Result of move before homing: {}", t);

    let t = mmd_processor.home().await;
    info!("Home done, t = {}", t);

    let _t = mmd_processor.move_to_relative(100).await;
    let _t = mmd_processor.move_to_relative(100).await;
    let _t = mmd_processor.move_to_relative(-100).await;
    let _t = mmd_processor.move_to_relative(100).await;
    let _t = mmd_processor.move_to(100).await;
    let _t = mmd_processor.move_to(500).await;
}
