#![no_std]
#![no_main]

use cortex_m::asm::delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use fugit::RateExtU32;
use panic_probe as _;
use pizzaro::common::tmc5160::{self, await_read_response, ReadableRegister, Reader as TmcReader};
use pizzaro::{
    mmd_motor42_nEN0, mmd_stepper57_dir, mmd_stepper57_nEN, mmd_stepper57_pwm_slice,
    mmd_stepper57_step, mmd_stepper57_step_channel, mmd_tmc_5160_addr, mmd_tmc_uart_rx,
    mmd_tmc_uart_tx,
};
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac::{self, Peripherals, UART0},
    sio::Sio,
    uart::{ReadError, Reader, Writer},
    watchdog::Watchdog,
};
use rp2040_hal::{
    entry,
    gpio::{
        bank0::{Gpio12, Gpio13, Gpio22},
        FunctionSioOutput, FunctionUart, Pin, Pins, PullUp,
    },
    uart::{StopBits, UartConfig, UartPeripheral},
};

/// The maximum PWM value (i.e. LED brightness) we want
const HIGH: u16 = 6000;

/// The TMC5160 registers that we care about.
pub struct TmcRegisters {
    gconf: tmc5160::reg::GCONF,
    //vactual: tmc5160::reg::VACTUAL,
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
    // let mut tmc0_en = mmd_motor42_nEN0!(pins)
    //     .into_pull_up_disabled()
    //     .into_push_pull_output();
    // let mut tmc1_en = mmd_motor42_nEN1!(pins)
    //     .into_pull_up_disabled()
    //     .into_push_pull_output();
    let mut tmc2_en = mmd_stepper57_nEN!(pins)
        .into_pull_up_disabled()
        .into_push_pull_output();
    // tmc0_en.set_high().unwrap();
    // tmc1_en.set_high().unwrap();
    tmc2_en.set_high().unwrap();

    let mut motor57_dir = mmd_stepper57_dir!(pins).into_push_pull_output();
    motor57_dir.set_low().unwrap();

    // init pwm:
    {
        let mut pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

        let pwm_57 = &mut mmd_stepper57_pwm_slice!(pwm_slices);
        pwm_57.set_top(HIGH);
        pwm_57.set_div_int(100);
        pwm_57.set_div_frac(0);

        let motor57_pulse = &mut mmd_stepper57_step_channel!(pwm_57);
        motor57_pulse.set_duty(HIGH / 2);
        motor57_pulse.output_to(mmd_stepper57_step!(pins));
        pwm_57.enable();
    }

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

    let mut gconf = tmc5160::reg::GCONF::default();
    // let vactual = tmc5160::reg::VACTUAL::ENABLED_STOPPED;
    let mut slaveaddr = tmc5160::reg::SLAVECONF::default();
    let mut ihold_run = tmc5160::reg::IHOLD_IRUN::default();
    let mut rampmode = tmc5160::reg::RAMPMODE::default();
    let mut vmax = tmc5160::reg::VMAX::default();
    let mut amax = tmc5160::reg::AMAX::default();
    let mut tpwmthrs = tmc5160::reg::TPWMTHRS::default();
    let mut chopconf = tmc5160::reg::CHOPCONF::default();
    ihold_run.set_ihold(5);
    ihold_run.set_irun(20);
    ihold_run.set_ihold_delay(1);
    rampmode.set(1);
    vmax.set(20000);
    amax.set(1000);
    tpwmthrs.set(500);
    chopconf.set_toff(5);
    chopconf.set_hend(1);
    chopconf.set_hstrt(4);
    chopconf.set_tbl(2);
    chopconf.set_mres(0);
    chopconf.set_chm(false);

    let addr = 5;
    slaveaddr.set_slaveaddr(addr);

    tmc5160::send_write_request(0, slaveaddr, &mut tmc_tx, &mut tmc_rx).unwrap();
    delay.delay_ms(1);

    tmc5160::send_write_request(addr, gconf, &mut tmc_tx, &mut tmc_rx).unwrap();
    delay.delay_ms(1);
    tmc5160::send_write_request(addr, chopconf, &mut tmc_tx, &mut tmc_rx).unwrap();
    delay.delay_ms(1);
    tmc5160::send_write_request(addr, ihold_run, &mut tmc_tx, &mut tmc_rx).unwrap();
    delay.delay_ms(1);

    tmc5160::send_write_request(addr, vmax, &mut tmc_tx, &mut tmc_rx).unwrap();
    delay.delay_ms(1);
    tmc5160::send_write_request(addr, amax, &mut tmc_tx, &mut tmc_rx).unwrap();
    delay.delay_ms(1);
    tmc5160::send_write_request(addr, rampmode, &mut tmc_tx, &mut tmc_rx).unwrap();
    delay.delay_ms(1);

    // enable stepper
    //tmc2_en.set_low().unwrap();

    //let tmc_regs = TmcRegisters { gconf };

    // The reader that we will use for interpreting TMC5160 UART responses.
    let mut reader = TmcReader::default();
    let mut buffer = [0u8; 32];
    loop {
        let ioin = read_reg::<tmc5160::reg::IOIN>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got IOIN: {}", Debug2Format(&ioin));
        let tstep = read_reg::<tmc5160::reg::TSTEP>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got TSTEP: {}", Debug2Format(&tstep));
        let gstat = read_reg::<tmc5160::reg::GSTAT>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got GSTAT: {}", Debug2Format(&gstat));
        let gconf = read_reg::<tmc5160::reg::GCONF>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got GCONF: {}", Debug2Format(&gconf),);
        let drv_stat = read_reg::<tmc5160::reg::DRV_STATUS>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got DRV_STATUS: {}", Debug2Format(&drv_stat));
        let ifcnt = read_reg::<tmc5160::reg::IFCNT>(&mut tmc_tx, &mut tmc_rx, &mut reader);
        info!("Got IFCNT: {}", Debug2Format(&ifcnt));

        delay.delay_ms(500);
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
    tmc5160::send_read_request::<R, _, _>(0, tx, rx).unwrap();

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
