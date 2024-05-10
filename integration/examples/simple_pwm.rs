#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_PwmPin;
use defmt::info;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use rp2040_hal::{entry, pac, Sio, Timer, Watchdog};
use rp2040_hal::clocks::init_clocks_and_plls;
use rp_pico::XOSC_CRYSTAL_FREQ;
use pizzaro::common::global_allocator::init_allocator;

#[entry]
fn main() -> ! {
    init_allocator();
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

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut led_pin = pins.gpio25.into_push_pull_output();
    let mut count = 0;

    let mut pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm = &mut pwm_slices.pwm4;
    pwm.default_config();
    pwm.set_ph_correct();
    // pwm.set_div_int();
    // pwm.set_counter()
    pwm.set_top(1023);
    pwm.channel_a.output_to(pins.gpio8);
    pwm.channel_a.set_duty(300);
    pwm.channel_b.output_to(pins.gpio9);
    pwm.channel_b.set_duty(300);
    pwm.channel_b.set_inverted();
    pwm.enable();

    loop {
        info!("in loop, count: {}", count);
        count += 1;
        led_pin.set_high().unwrap();
        timer.delay_ms(1000);
        led_pin.set_low().unwrap();
        timer.delay_ms(1000);
    }
}
