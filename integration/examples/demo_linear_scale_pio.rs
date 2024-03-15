#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;

use cortex_m::asm::delay;
use defmt::info;
use fugit::ExtU64;
use rp2040_hal::{entry, pac, Timer, Watchdog};
use rp2040_hal::clocks::init_clocks_and_plls;
use rp_pico::XOSC_CRYSTAL_FREQ;

use pizzaro::common::async_initialization;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{Delay, init_global_timer};
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::hpd::hpd_misc::LinearScale;
use pizzaro::hpd::linear_scale::{init_quadrature_encoder, read_linear_scale_with_pio};

struct GlobalContainer {
    linear_scale: Option<LinearScale>,
}
static mut GLOBAL_CONTAINER: GlobalContainer = GlobalContainer { linear_scale: None };
const LINEAR_SCALE_PIN_A_ID: u8 = 10;

#[entry]
fn main() -> ! {
    async_initialization();
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
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

    unsafe {
        GLOBAL_CONTAINER.linear_scale.replace(LinearScale::new());
    }
    let linear_scale_rc0 = unsafe { GLOBAL_CONTAINER.linear_scale.as_mut().unwrap() };
    let linear_scale_rc1 = unsafe { GLOBAL_CONTAINER.linear_scale.as_mut().unwrap() };
    {
        let rx = init_quadrature_encoder(LINEAR_SCALE_PIN_A_ID, pac.PIO0, &mut pac.RESETS).expect("init pio error");
        spawn_task(read_linear_scale_with_pio(rx, linear_scale_rc0));
    }

    {
        spawn_task(log_linear_scale(linear_scale_rc1));
    }

    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

pub async fn log_linear_scale(linear_scale: &LinearScale) {
    loop {
        info!("Current relative position of the linear scale: {:?}", linear_scale.get_rel_position());
        Delay::new(500.millis()).await;
    }
}