#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;
use pizzaro::blinky_led;
use pizzaro::common::led_controller::MyLED;
use pizzaro::common::ws2812_bitbang::Ws2812;
// use rp2040_hal::pio::PIOExt;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    sio::Sio,
    watchdog::Watchdog,
};
use rp2040_hal::{entry, pac, Timer};

macro_rules! create_gpio_pool {
    ($pins:expr, $($gpio:ident),*) => {
        [
            $($pins.$gpio.into_push_pull_output().into_dyn_pin()),*
        ]
    };
}

// macro_rules! create_gpio_pool {
//     ($pins:expr, $($gpio:ident),*) => {{
// 	[
//             $($pins.$gpio.into_push_pull_output().into_dyn_pin()),*
//         ];
//     }};
// }

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    // let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
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

    let mut my_led = MyLED::new(
        (5, 5, 5).into(),
        (0, 0, 0).into(),
        Ws2812::new(blinky_led!(pins).into_function().into_dyn_pin()),
    );

    // let mut gpio_pool = create_gpio_pool!(
    //     pins,
    //     gpio0, gpio1, gpio2, gpio3, gpio4, gpio5, gpio6, gpio7, gpio8, gpio9, gpio10, gpio11,
    //     gpio12, gpio13, gpio14, gpio15, //gpio16,
    //     gpio17, gpio18, gpio19, gpio20, gpio21, gpio22, gpio23, gpio24, gpio25, gpio26, gpio27,
    //     gpio28, gpio29
    // );

    let mut gpio_pool = create_gpio_pool!(pins, gpio4, gpio5, gpio21);
    let mut idx = 0;
    loop {
        //info!("toggle {}", gpio_pool[idx]);
        my_led.ledon();
        gpio_pool[idx].set_high().unwrap();
        delay.delay_ms(200);

        my_led.ledoff();
        gpio_pool[idx].set_low().unwrap();
        delay.delay_ms(200);
        idx = (idx + 1) % gpio_pool.len();
    }
}
