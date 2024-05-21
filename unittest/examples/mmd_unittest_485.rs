#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use fugit::RateExtU32;
use panic_probe as _;
use pizzaro::{
    blinky_led,
    common::{led_controller::MyLED, ws2812_bitbang::Ws2812},
    mmd_485_dir, mmd_sys_rx, mmd_sys_tx, mmd_uart,
};

use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry,
    gpio::{FunctionUart, Pins},
    pac,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
};

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

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let led_pin = blinky_led!(pins).into_function().into_dyn_pin();
    let mut myled = MyLED::new((10, 10, 10).into(), (0, 0, 0).into(), Ws2812::new(led_pin));

    // Initialize UART
    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        mmd_sys_tx!(pins).into_function::<FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        mmd_sys_rx!(pins).into_function::<FunctionUart>(),
    );
    let uart = UartPeripheral::new(mmd_uart!(pac), uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    let mut uart_dir = mmd_485_dir!(pins).into_push_pull_output().into_dyn_pin();
    uart_dir.set_low().unwrap();
    // let mut comm = UartComm::new(&mut uart, &mut uart_dir, 128);
    let mut led_on_stat = false;
    loop {
        let mut buffer: [u8; 32] = [0; 32];
        match nb::block!(uart.read_raw(&mut buffer)) {
            Ok(_) => {
                info!("buffer:{}", buffer);
            }
            Err(_) => {
                error!("recv error");
            }
        }

        led_on_stat = !led_on_stat;
        if led_on_stat {
            myled.ledon();
        } else {
            myled.ledoff();
        }

        delay.delay_ms(200);
    }
}
