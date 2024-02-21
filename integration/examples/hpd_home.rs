#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;

use cortex_m::asm::delay;
use defmt::info;
use rp2040_hal::{entry, pac, Sio, Timer, Watchdog};
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::multicore::{Multicore, Stack};
use rp_pico::XOSC_CRYSTAL_FREQ;

use pizzaro::common::async_initialization;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::init_global_timer;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::hpd::hpd_misc::{LinearScale, MOTOR150_PWM_TOP, PwmMotor};
use pizzaro::hpd::hpd_processor::HpdProcessor;
use pizzaro::hpd::linear_scale::{core1_task, read_and_update_linear_scale};

struct GlobalContainer {
    linear_scale: Option<LinearScale>,
}
static mut GLOBAL_CONTAINER: GlobalContainer = GlobalContainer { linear_scale: None };

static mut CORE1_STACK: Stack<4096> = Stack::new();

#[entry]
fn main() -> ! {
    async_initialization();
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

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

    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task()
    });

    unsafe {
        GLOBAL_CONTAINER.linear_scale.replace(LinearScale::new());
    }
    let linear_scale_rc0 = unsafe { GLOBAL_CONTAINER.linear_scale.as_mut().unwrap() };
    let linear_scale_rc1 = unsafe { GLOBAL_CONTAINER.linear_scale.as_mut().unwrap() };
    {
        // Start scale
        spawn_task(read_and_update_linear_scale(sio.fifo, linear_scale_rc0));
    }

    {
        // Start motor150
        let pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
        let mut pwm = pwm_slices.pwm0;
        pwm.set_ph_correct();
        pwm.set_top(MOTOR150_PWM_TOP);
        pwm.enable();
        pwm.channel_a.output_to(pins.gpio16);
        pwm.channel_b.output_to(pins.gpio17);
        pwm.channel_b.set_inverted();

        let processor = HpdProcessor::new(
            linear_scale_rc1,
            PwmMotor::new(pins.gpio18.into_push_pull_output(), pwm),
        );
        spawn_task(hpd_home(processor));
    }

    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

async fn hpd_home(mut processor: HpdProcessor) {
    let mut t = processor.home().await.unwrap();
    info!("xfguo: position after homing: {}, start move to relative 10000", t);
    t = processor.move_to_relative(10000).await.unwrap();
    info!("xfguo: position 2: {}", t);
    t = processor.move_to_relative(10000).await.unwrap();
    info!("xfguo: position 3: {}", t);
    t = processor.move_to_relative(-10000).await.unwrap();
    info!("xfguo: position 4: {}", t);
    t = processor.move_to(30000).await.unwrap();
    info!("xfguo: position 5: {}", t);
    t = processor.move_to(10000).await.unwrap();
    info!("xfguo: position 6: {}", t);
}
