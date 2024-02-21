#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use cortex_m::asm::delay;
use defmt_rtt as _;
use panic_probe as _;

use defmt::info;
use embedded_hal::digital::v2::InputPin;
use fugit::ExtU64;
use pizzaro::common::async_initialization;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, Delay};
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::hpd_processor::{HpdProcessor, LinearScale, PwmMotor};
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::multicore::{Multicore, Stack};
use rp2040_hal::sio::SioFifo;
use rp2040_hal::{entry, pac, Sio, Timer, Watchdog};
use rp_pico::{hal, XOSC_CRYSTAL_FREQ};

struct GlobalContainer {
    linear_scale: Option<LinearScale>,
}
static mut GLOBAL_CONTAINER: GlobalContainer = GlobalContainer { linear_scale: None };

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task() -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };

    let mut sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let gpio10 = pins.gpio10.into_floating_input();
    let gpio11 = pins.gpio11.into_floating_input();
    let (mut x, mut y) = (true, false);
    let mut pos = 0i32;
    let mut count = 0;
    loop {
        let new_x = gpio10.is_high().unwrap_or(false);
        let new_y = gpio11.is_high().unwrap_or(true);
        match (x, y, new_x, new_y) {
            (false, false, true, false)
            | (false, true, false, false)
            | (true, false, true, true)
            | (true, true, false, true) => {
                // increment
                pos += 1;
            }
            (false, false, false, true)
            | (false, true, true, true)
            | (true, false, false, false)
            | (true, true, true, false) => {
                // decrement
                pos -= 1;
            }
            (_, _, _, _) => {}
        }
        (x, y) = (new_x, new_y);
        count += 1;
        if count % 500 == 13 {
            sio.fifo.write(pos as u32);
        }
    }
}

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

async fn read_and_update_linear_scale(mut fifo: SioFifo, linear_scale: &mut LinearScale) {
    loop {
        {
            let mut last_pos: Option<i32> = None;
            // let mut count = 0;
            while let Some(pos) = fifo.read() {
                // count += 1;
                last_pos = Some(pos as i32);
            }
            if let Some(t) = last_pos {
                // info!("count = {}, pos = {}", count, t);
                linear_scale.update_position(t);
            }
        }
        Delay::new(1.millis()).await;
    }
}

const MOTOR150_PWM_TOP: u16 = 2000;
