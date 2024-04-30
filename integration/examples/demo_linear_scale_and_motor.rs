#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;

use cortex_m::asm::delay;
use defmt::info;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::PwmPin;
use fugit::ExtU64;
use hal::gpio::DynPinId;
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::gpio::{FunctionSio, Pin, PullDown, SioOutput};
use rp2040_hal::multicore::{Multicore, Stack};
use rp2040_hal::pwm::{FreeRunning, Pwm0, Slice};
use rp2040_hal::sio::SioFifo;
use rp2040_hal::{entry, pac, Sio, Timer, Watchdog};
use rp_pico::{hal, XOSC_CRYSTAL_FREQ};

use pizzaro::common::async_initialization;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, Delay};
use pizzaro::common::rp2040_timer::Rp2040Timer;

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task() -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };

    let mut sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let ols_a = pins.gpio10.into_floating_input();
    let ols_b = pins.gpio11.into_floating_input();
    let (mut x, mut y) = (true, false);
    let mut pos = 0i32;
    let mut count = 0;
    loop {
        let new_x = ols_a.is_high().unwrap_or(false);
        let new_y = ols_b.is_high().unwrap_or(true);
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
        if count % 100_000 == 13 {
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

    let pins =
        rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || core1_task());

    {
        // Start scale
        spawn_task(read_linear_scale(sio.fifo));
    }

    {
        // Start motor150
        let pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
        let mut pwm = pwm_slices.pwm0;
        pwm.set_ph_correct();
        pwm.set_top(MOTOR150_PWM_TOP);
        pwm.enable();
        pwm.channel_a.output_to(pins.gpio0);
        pwm.channel_b.output_to(pins.gpio1);
        pwm.channel_b.set_inverted();
        spawn_task(run_motor_with_different_speed(
            pins.gpio2.into_push_pull_output().into_dyn_pin(),
            pwm,
        ));
    }

    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

async fn read_linear_scale(mut fifo: SioFifo) {
    loop {
        if let Some(pos) = fifo.read() {
            info!("xfguo: pos = {}", pos as i32);
        }
        Delay::new(1.millis()).await;
    }
}

const MOTOR150_PWM_TOP: u16 = 2000;

async fn run_motor_with_different_speed(
    mut motor_enable_pin: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
    mut pwm: Slice<Pwm0, FreeRunning>,
) {
    let mut duty = 50i32;
    let mut step = 2i32;
    motor_enable_pin.set_low().unwrap();
    loop {
        if duty <= 0 || duty >= 100 {
            step *= -1;
        }
        duty += step;
        let duty_scaled = (MOTOR150_PWM_TOP as i32) * duty / 100;
        info!("heartbeat: duty = {}, duty_scaled = {}", duty, duty_scaled);
        pwm.channel_a.set_duty(duty_scaled as u16);
        pwm.channel_b.set_duty(duty_scaled as u16);
        Delay::new(1000.millis()).await;
    }
}
