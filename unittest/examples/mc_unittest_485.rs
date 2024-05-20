#![no_std]
#![no_main]
extern crate alloc;

use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use fugit::RateExtU32;
use futures::TryFutureExt;
use generic::{atomi_error::AtomiError, atomi_proto::AtomiProto, mmd_status::MmdStatus};
use panic_probe as _;
use pizzaro::{
    blinky_led,
    common::{
        executor_inplace::executor_inplace, global_allocator::init_allocator,
        global_timer::AtomiDuration, led_controller::MyLED, uart_comm::UartComm,
        ws2812_bitbang::Ws2812,
    },
    mc_485_dir, mc_sys_rx, mc_sys_tx, mc_uart,
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
    init_allocator();
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
        mc_sys_tx!(pins).into_function::<FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        mc_sys_rx!(pins).into_function::<FunctionUart>(),
    );
    let mut uart = UartPeripheral::new(mc_uart!(pac), uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    let mut uart_dir = mc_485_dir!(pins).into_push_pull_output().into_dyn_pin();
    // let mut comm = UartComm::new(&mut uart, &mut uart_dir, 128);
    uart_dir.set_high();
    loop {
        //comm.send(AtomiProto::Mmd(generic::atomi_proto::MmdCommand::MmdPing)).unwrap();
        uart.write_full_blocking(b"hello");
        info!("ping MMD...");
        delay.delay_ms(200);
    }
    // loop {
    //     comm.send(AtomiProto::Mmd(generic::atomi_proto::MmdCommand::MmdPing)).unwrap();
    //     info!("ping MMD...");
    //     let mmd_resp: Result<AtomiProto, AtomiError> = executor_inplace(
    //         comm.recv_timeout(AtomiDuration::millis(100))
    //             .map_err(|_| AtomiError::MmdUnavailable(MmdStatus::Unavailable)),
    //     );
    //     info!("MMD resp:{}", mmd_resp);
    //     myled.ledon();
    //     delay.delay_ms(200);

    //     comm.send(AtomiProto::Hpd(generic::atomi_proto::HpdCommand::HpdPing)).unwrap();
    //     info!("ping HPD...");
    //     let hpd_resp: Result<AtomiProto, AtomiError> = executor_inplace(
    //         comm.recv_timeout(AtomiDuration::millis(100)).map_err(|_| AtomiError::HpdUnavailable),
    //     );
    //     info!("HPD resp:{}", hpd_resp);
    //     myled.ledoff();
    //     delay.delay_ms(200);
    // }
}
