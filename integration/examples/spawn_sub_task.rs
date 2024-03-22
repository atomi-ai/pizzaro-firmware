#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;

use defmt;
use defmt::info;
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::{entry, pac, Sio, Timer, Watchdog};
use rp_pico::XOSC_CRYSTAL_FREQ;

use pizzaro::common::async_initialization;
use pizzaro::common::executor::{dump_executor_status, spawn_task, start_global_executor};
use pizzaro::common::global_timer::init_global_timer;
use pizzaro::common::rp2040_timer::Rp2040Timer;

#[entry]
fn main() -> ! {
    info!("Start program");
    async_initialization();
    let mut pac = pac::Peripherals::take().unwrap();
    let _sio = Sio::new(pac.SIO);
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

    spawn_task(dump_executor_status());
    spawn_task(one());
    start_global_executor();

    loop {}
}

async fn one() {
    info!("one");
    spawn_task(two());
}

async fn two() {
    info!("two");
}
