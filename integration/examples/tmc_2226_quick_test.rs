#![no_std]
#![no_main]
use cortex_m::asm::delay;
use defmt::*;
use defmt_rtt as _;
use fugit::RateExtU32;
use panic_probe as _;
use pizzaro::common::tmc2209::{self, await_read_response, ReadableRegister, Reader as TmcReader};
use pizzaro::{mmd_motor42_nEN0, mmd_motor42_nEN1, mmd_tmc_uart_rx, mmd_tmc_uart_tx};
use rp2040_hal::{
    entry,
    gpio::{
        bank0::{Gpio12, Gpio13, Gpio22},
        FunctionSioOutput, FunctionUart, Pin, Pins, PullUp,
    },
    pac::UART0,
    uart::{Reader, StopBits, UartConfig, UartPeripheral, Writer},
};

use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

/// The TMC2209 registers that we care about.
pub struct TmcRegisters {
    gconf: tmc2209::reg::GCONF,
    vactual: tmc2209::reg::VACTUAL,
}

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

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let uart_tx: Pin<Gpio12, FunctionUart, PullUp> = mmd_tmc_uart_tx!(pins).reconfigure();
    let uart_rx: Pin<Gpio13, FunctionUart, PullUp> = mmd_tmc_uart_rx!(pins).reconfigure();
    let mut tmc0_en = mmd_motor42_nEN0!(pins)
        .into_pull_up_disabled()
        .into_push_pull_output();
    let mut tmc1_en = mmd_motor42_nEN1!(pins)
        .into_pull_up_disabled()
        .into_push_pull_output();
    // let mut tmc2_en = mmd_stepper57_nEN!(pins)
    //     .into_pull_up_disabled()
    //     .into_push_pull_output();
    // let mut tmc_5160_addr = mmd_tmc_5160_addr!(pins)
    //     .into_pull_up_disabled()
    //     .into_push_pull_output();
    tmc0_en.set_high().unwrap();
    tmc1_en.set_high().unwrap();
    // tmc2_en.set_high().unwrap();
    // tmc_5160_addr.set_high().unwrap();

    let uart = UartPeripheral::new(pac.UART0, (uart_tx, uart_rx), &mut pac.RESETS)
        .enable(
            UartConfig::new(
                RateExtU32::Hz(115_200),
                rp2040_hal::uart::DataBits::Eight,
                None,
                StopBits::One,
            ),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    let (mut tmc_rx, mut tmc_tx) = uart.split();

    let mut gconf = tmc2209::reg::GCONF::default();
    let vactual = tmc2209::reg::VACTUAL::ENABLED_STOPPED;
    gconf.set_pdn_disable(true);
    let mut ihold_irun = tmc2209::reg::IHOLD_IRUN::default();
    ihold_irun.set_ihold(25);
    ihold_irun.set_irun(31);
    ihold_irun.set_ihold_delay(7);

    let addr = 2;

    tmc2209::send_write_request(addr, gconf, &mut tmc_tx, &mut tmc_rx).unwrap();
    tmc2209::send_write_request(addr, vactual, &mut tmc_tx, &mut tmc_rx).unwrap();
    tmc2209::send_write_request(addr, ihold_irun, &mut tmc_tx, &mut tmc_rx).unwrap();

    let mut tmc_regs = TmcRegisters { gconf, vactual };

    // The reader that we will use for interpreting TMC2209 UART responses.
    let mut reader = TmcReader::default();
    let mut buffer = [0u8; 32];
    let mut velocity = 100;
    tmc_regs.vactual.set(velocity);
    tmc2209::send_write_request(addr, tmc_regs.vactual, &mut tmc_tx, &mut tmc_rx).unwrap();

    loop {
        tmc2209::send_write_request(addr, tmc_regs.vactual, &mut tmc_tx, &mut tmc_rx).unwrap();
        tmc2209::send_write_request(addr, tmc_regs.gconf, &mut tmc_tx, &mut tmc_rx).unwrap();
        let ioin = read_reg::<tmc2209::reg::IOIN>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got IOIN: {}", Debug2Format(&ioin));
        let tstep = read_reg::<tmc2209::reg::TSTEP>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got TSTEP: {}", Debug2Format(&tstep));
        let gstat = read_reg::<tmc2209::reg::GSTAT>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got GSTAT: {}", Debug2Format(&gstat));
        let gconf = read_reg::<tmc2209::reg::GCONF>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got GCONF: {}", Debug2Format(&gconf),);
        let drv_stat = read_reg::<tmc2209::reg::DRV_STATUS>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got DRV_STATUS: {}", Debug2Format(&drv_stat));
        let ifcnt = read_reg::<tmc2209::reg::IFCNT>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got IFCNT: {}", Debug2Format(&ifcnt));
        delay.delay_ms(500);

        if let Some(gstat) = gstat {
            if gstat.uv_cp() {
                tmc2209::send_write_request(addr, gstat, &mut tmc_tx, &mut tmc_rx).unwrap();
            }
        }
    }
}

type UartPinsType = (
    Pin<Gpio12, FunctionUart, PullUp>,
    Pin<Gpio13, FunctionUart, PullUp>,
);

fn read_reg<R: ReadableRegister>(
    tx: &mut Writer<UART0, UartPinsType>,
    rx: &mut Reader<UART0, UartPinsType>,
    tmc_reader: &mut TmcReader,
) -> Option<R> {
    let mut buffer = [0u8; 32];
    tmc2209::send_read_request::<R, _, _>(0, tx, rx).unwrap();

    let mut timeout = 50000u64;
    loop {
        match rx.read_raw(&mut buffer) {
            Ok(bytes) => {
                // info!("bytes:{}, buffer:{}", bytes, buffer);
                timeout = 50000u64;
                let (processed_bytes, tmc_response) = tmc_reader.read_response(&buffer[..bytes]);
                // info!(
                //     "processed bytes: {}, recv bytes: {}",
                //     processed_bytes, bytes
                // );
                let res = if let Some(response) = tmc_response {
                    if !response.crc_is_valid() {
                        error!("Received invalid response!");
                        return None;
                    }
                    // info!("Received valid response: {}", Debug2Format(&response));
                    match response.reg_addr() {
                        Ok(addr) => {
                            // info!("xfguo: addr = {}, R::ADDR = {}", Debug2Format(&addr), Debug2Format(&R::ADDRESS));
                            if addr == R::ADDRESS {
                                let reg = response.register::<R>().unwrap();
                                // info!("xfguo: {}: {}", Debug2Format(&R::ADDRESS), Debug2Format(&reg));
                                Some(reg)
                            } else {
                                None
                            }
                        }
                        _ => None,
                    }
                } else {
                    None
                };
                if res.is_none() {
                    continue;
                }
                return res;
            }
            Err(nb::Error::WouldBlock) => {
                if timeout > 0 {
                    timeout -= 1;
                    delay(10);
                    continue;
                } else {
                    error!("timeout in reading");
                    return None;
                }
            }
            Err(e) => {
                error!("Errors in read data");
                return None;
            }
        }
    }
}
