#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use panic_probe as _;
use pizzaro::{
    blinky_led,
    bsp::board_mmd_release_sb::MmdLimitSwitch0,
    common::{led_controller::MyLED, ws2812_bitbang::Ws2812},
    mmd_limit0, mmd_limit1,
};
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry, pac,
    sio::Sio,
    watchdog::Watchdog,
};

/// The maximum PWM value (i.e. LED brightness) we want
// const HIGH: u16 = 60000;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins =
        rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let mut my_led = MyLED::new(
        (5, 5, 5).into(),
        (0, 0, 0).into(),
        Ws2812::new(blinky_led!(pins).into_function().into_dyn_pin()),
    );
    let mut limit_left_pin = mmd_limit0!(pins).into_floating_input();
    let mut limit_right_pin = mmd_limit1!(pins).into_floating_input();
    let mut delay_interval: u32 = 500;
    loop {
        my_led.ledon();
        delay.delay_ms(delay_interval);

        my_led.ledoff();
        delay.delay_ms(delay_interval);
        let (left_on, right_on) =
            (!limit_left_pin.is_high().unwrap(), !limit_right_pin.is_high().unwrap());
        if left_on || right_on {
            delay_interval = 100;
        } else {
            delay_interval = 500;
        }
        info!("left: {}   right:{}", limit_left_pin.is_high(), limit_right_pin.is_high());
    }
}
