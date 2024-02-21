#![no_std]
#![no_main]


extern crate alloc;

use panic_probe as _;
use defmt_rtt as _;

use alloc::boxed::Box;
use pizzaro::common::global_timer::{Delay, init_global_timer};
use defmt::{error, info};
use fugit::ExtU64;
use integration::mock_stepper::{homing, MockStepper, stepper_tick};
use integration::mock_stepper::x::StepperAdapter;
use rp2040_hal::{entry, pac, Timer, Watchdog};
use rp2040_hal::clocks::init_clocks_and_plls;
use rp_pico::XOSC_CRYSTAL_FREQ;
use pizzaro::common::async_initialization;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_status::{FutureType, get_status};
use pizzaro::common::global_status::FutureStatus::Completed;
use pizzaro::common::rp2040_timer::Rp2040Timer;

struct GlobalContainer {
    stepper: Option<MockStepper>,
}
static mut GLOBAL_CONTAINER: GlobalContainer = GlobalContainer {
    stepper: None,
};

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

    {
        unsafe { GLOBAL_CONTAINER.stepper.replace(MockStepper::new(-100, 200)); }
        let stepper_rc0 = unsafe { StepperAdapter(GLOBAL_CONTAINER.stepper.as_mut().unwrap()) };
        let stepper_rc1 = unsafe { StepperAdapter(GLOBAL_CONTAINER.stepper.as_mut().unwrap()) };
        spawn_task(stepper_tick(stepper_rc0));
        spawn_task(homing(stepper_rc1, 10, 1));
    }

    // Spawn a test future.
    spawn_task(demo_test());
    start_global_executor();

    loop {
        // won't execute the code here.
    }
}

async fn demo_test() {
    for _ in 0..10 {
        Delay::new(1.secs()).await;

        if let Some(status) = get_status(FutureType::StepperHoming) {
            if matches!(status, Completed) {
                info!("[TEST] Done");
                return;
            }
        }
    }
    error!("[TEST] Failed");
}