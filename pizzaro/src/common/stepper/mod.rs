use crate::common::tmc2209;
use core::sync::atomic::AtomicBool;
use embedded_hal_nb::serial::{Read, Write};
use generic::atomi_error::AtomiError;

pub mod classic_stepper;
#[allow(async_fn_in_trait)]
pub mod classic_stepper_driver;
pub mod classic_stepper_processor;

pub static GLOBAL_STEPPER_STOP: AtomicBool = AtomicBool::new(false);

pub static GLOBAL_DTU_STEPPER_STOP: AtomicBool = AtomicBool::new(false);

pub fn init_tmc_parameters<R: Read<u8>, W: Write<u8>>(
    irun: u8,
    ihold: u8,
    stallguard_thres: Option<u32>,
    tmc_addr: u8,
    mut uart_rx: R,
    mut uart_tx: W,
) -> Result<(), AtomiError> {
    let mut gconf = tmc2209::reg::GCONF::default();
    let sgthres = if let Some(sgt) = stallguard_thres {
        // enter stealth chop mode to enable sensorless
        gconf.set_en_spread_cycle(false);
        Some(tmc2209::reg::SGTHRS(sgt))
    } else {
        // enter spread cycle mode
        gconf.set_en_spread_cycle(true);
        None
    };
    let mut clear_gstat = tmc2209::reg::GSTAT::default();
    clear_gstat.clear_uv_cp(true);
    clear_gstat.clear_reset(true);
    clear_gstat.clear_drv_err(true);
    let mut ihold_run = tmc2209::reg::IHOLD_IRUN::default();
    let mut tpwmthrs = tmc2209::reg::TPWMTHRS::default();
    let mut chopconf = tmc2209::reg::CHOPCONF::default();

    ihold_run.set_ihold(ihold);
    ihold_run.set_irun(irun);
    ihold_run.set_ihold_delay(1);
    tpwmthrs.set(500);
    chopconf.set_toff(5);
    chopconf.set_hend(1);
    chopconf.set_hstrt(4);
    chopconf.set_tbl(2);
    chopconf.set_mres(0);

    tmc2209::send_write_request(tmc_addr, gconf, &mut uart_tx, &mut uart_rx)
        .map_err(|_| AtomiError::DtuTMCWriteRequestError)?;
    tmc2209::send_write_request(tmc_addr, clear_gstat, &mut uart_tx, &mut uart_rx)
        .map_err(|_| AtomiError::DtuTMCWriteRequestError)?;
    tmc2209::send_write_request(tmc_addr, chopconf, &mut uart_tx, &mut uart_rx)
        .map_err(|_| AtomiError::DtuTMCWriteRequestError)?;
    tmc2209::send_write_request(tmc_addr, ihold_run, &mut uart_tx, &mut uart_rx)
        .map_err(|_| AtomiError::DtuTMCWriteRequestError)?;
    if let Some(sgthres) = sgthres {
        tmc2209::send_write_request(tmc_addr, sgthres, &mut uart_tx, &mut uart_rx)
            .map_err(|_| AtomiError::DtuTMCWriteRequestError)?;
    }
    Ok(())
}
