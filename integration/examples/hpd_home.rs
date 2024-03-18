#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;

use cortex_m::asm::delay;
use defmt::info;
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};

use pizzaro::bsp::config::{HPD_BR_DRIVER_N_EN, HPD_BR_THRESHOLD, REVERT_HPD_BR_DIRECTION};
use pizzaro::common::async_initialization;
use pizzaro::common::brush_motor::BrushMotor;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, Delay};
use pizzaro::common::led_controller::{blinky_smart_led, MyLED};
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::hpd::hpd_misc::{LinearScale, MOTOR150_PWM_TOP};
use pizzaro::hpd::linear_bull_processor::LinearBullProcessor;
use pizzaro::hpd::linear_scale::{core1_task, read_and_update_linear_scale};
use pizzaro::{hpd_br_nEN, hpd_br_pwm_a, hpd_br_pwm_b, smart_led};

use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::fugit::ExtU64;
use rp2040_hal::gpio::{DynPinId, FunctionSio, Pin, PullDown, SioOutput};
use rp2040_hal::multicore::{Multicore, Stack};
use rp2040_hal::pio::PIOExt;
use rp2040_hal::pwm::SliceId;
use rp2040_hal::{entry, pac, Clock, Sio, Timer, Watchdog};

use rp_pico::XOSC_CRYSTAL_FREQ;
use ws2812_pio::Ws2812Direct;

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
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
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
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || core1_task());

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
        pwm.channel_a.output_to(hpd_br_pwm_a!(pins));
        pwm.channel_b.output_to(hpd_br_pwm_b!(pins));
        pwm.channel_b.set_inverted();

        let processor = LinearBullProcessor::new(
            linear_scale_rc1,
            BrushMotor::new(
                hpd_br_nEN!(pins).into_push_pull_output().into_dyn_pin(),
                pwm,
                HPD_BR_THRESHOLD,
                REVERT_HPD_BR_DIRECTION,
                HPD_BR_DRIVER_N_EN,
            ),
        );
        spawn_task(hpd_home(processor));
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

async fn hpd_home<S: SliceId, E: StatefulOutputPin>(mut processor: LinearBullProcessor<S, E>) {
    let mut t = processor.home().await.unwrap();
    info!(
        "xfguo: position after homing: {}, start move to relative 10000",
        t
    );
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

#[allow(dead_code)]
async fn blinky(mut led: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>) {
    loop {
        led.set_low().unwrap();
        Delay::new(500.millis()).await;
        led.set_high().unwrap();
        Delay::new(500.millis()).await;
    }
}
