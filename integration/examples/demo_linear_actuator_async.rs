#![no_std]
#![no_main]


extern crate alloc;

use panic_probe as _;
use defmt_rtt as _;

use alloc::boxed::Box;
use common::cap_displacement_processor::{CapDisplacementProcessor, CdpAdapter, monitor_cap_displacement_spi, read_cdp_result};
use common::global_timer::{Delay, init_global_timer};
use defmt::{error, info};
use fugit::ExtU64;
use integration::mock_stepper::{homing, MockStepper, stepper_tick};
use integration::mock_stepper::x::StepperAdapter;
use rp2040_hal::{entry, pac, Sio, Spi, Timer, Watchdog};
use rp2040_hal::clocks::init_clocks_and_plls;
use rp_pico::XOSC_CRYSTAL_FREQ;
use rp2040_hal::gpio::{FunctionSpi, PullNone, PullUp};
use common::async_initialization;
use common::executor::{spawn_task, start_global_executor};
use common::global_status::{FutureType, get_status};
use common::global_status::FutureStatus::Completed;
use common::rp2040_timer::Rp2040Timer;

struct GlobalContainer {
    stepper: Option<MockStepper>,
    cdp: Option<CapDisplacementProcessor>,
}
static mut GLOBAL_CONTAINER: GlobalContainer = GlobalContainer {
    stepper: None,
    cdp: None,
};

#[entry]
fn main() -> ! {
    async_initialization();
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

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

    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    init_global_timer(Box::new(Rp2040Timer::new(timer)));

    start_global_executor();
    {
        unsafe { GLOBAL_CONTAINER.stepper.replace(MockStepper::new(-100, 200)); }
        let stepper_rc0 = unsafe { StepperAdapter(GLOBAL_CONTAINER.stepper.as_mut().unwrap()) };
        let stepper_rc1 = unsafe { StepperAdapter(GLOBAL_CONTAINER.stepper.as_mut().unwrap()) };
        spawn_task(stepper_tick(stepper_rc0));
        spawn_task(homing(stepper_rc1, 10, 1));
    }

    {
        // Init SPI for the Cap Displacement (容栅尺)
        let spi_mosi = pins.gpio15.into_function::<FunctionSpi>().into_pull_type::<PullUp>();
        let spi_miso = pins.gpio12.into_function::<FunctionSpi>().into_pull_type::<PullNone>();
        let spi_sclk = pins.gpio14.into_function::<FunctionSpi>().into_pull_type::<PullNone>();
        let spi = Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));
        let spi = spi.init_slave(&mut pac.RESETS, embedded_hal::spi::MODE_3);

        // Init a data processor for the cap displacement (容栅尺)
        let cdp = CapDisplacementProcessor::new();
        unsafe { GLOBAL_CONTAINER.cdp.replace(cdp); }
        let cdp_0 = unsafe { CdpAdapter::new(GLOBAL_CONTAINER.cdp.as_mut().unwrap()) };
        let cdp_1 = unsafe { CdpAdapter::new(GLOBAL_CONTAINER.cdp.as_mut().unwrap()) };
        spawn_task(monitor_cap_displacement_spi(spi, cdp_0));
        spawn_task(read_cdp_result(cdp_1));
    }

    // Spawn a test future.
    spawn_task(demo_test());

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