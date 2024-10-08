#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;
use pizzaro::common::led_controller::MyLED;
use rp2040_hal::pio::PIOExt;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    sio::Sio,
    watchdog::Watchdog,
};
use rp2040_hal::{entry, pac, Timer};
use ws2812_pio::Ws2812Direct;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
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

    // let mut FC0_DELAY = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut _timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins =
        rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let mut test_gpio_pin = pins.gpio13.into_push_pull_output();

    let mut my_led = MyLED::new(
        (5, 5, 5).into(),
        (0, 0, 0).into(),
        Ws2812Direct::new(
            pins.gpio16.into_function().into_dyn_pin(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        ),
    );

    loop {
        my_led.ledon();
        test_gpio_pin.set_high().unwrap();
        delay.delay_ms(200);

        my_led.ledoff();
        test_gpio_pin.set_low().unwrap();
        delay.delay_ms(200);
    }
}
