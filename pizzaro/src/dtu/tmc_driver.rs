use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
use embedded_hal::serial::{Read, Write};
use fugit::ExtU64;
use generic::atomi_error::AtomiError;

use crate::common::{global_timer::AsyncDelay, tmc2209};

pub struct TmcDriver<
    OP1: StatefulOutputPin,
    OP2: OutputPin,
    OP3: OutputPin,
    D: AsyncDelay,
    U: Write<u8>,
    X: Read<u8>,
> {
    enable_pin: OP1,
    dir_pin: OP2,
    step_pin: OP3,
    revert_dir: bool,

    speed: u32,
    wait_period: u64,
    async_delay: D,
    uart_tx: U,
    uart_rx: X,
    tmc_addr: u8,
}

impl<
        OP1: StatefulOutputPin,
        OP2: OutputPin,
        OP3: OutputPin,
        D: AsyncDelay,
        U: Write<u8>,
        X: Read<u8>,
    > TmcDriver<OP1, OP2, OP3, D, U, X>
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        enable_pin: OP1,
        dir_pin: OP2,
        step_pin: OP3,
        async_delay: D,
        revert_dir: bool,
        uart_tx: U,
        uart_rx: X,
        tmc_addr: u8,
    ) -> Self {
        TmcDriver {
            enable_pin,
            dir_pin,
            step_pin,
            revert_dir,
            async_delay,
            speed: 0,
            wait_period: 0,
            uart_tx,
            uart_rx,
            tmc_addr,
        }
    }

    pub fn init_tmc_parameters(
        &mut self,
        irun: u8,
        ihold: u8,
        stallguard_thres: Option<u32>,
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
        // let vactual = tmc2209::reg::VACTUAL::ENABLED_STOPPED;
        // let slaveaddr = tmc2209::reg::SLAVECONF::default();
        let mut ihold_run = tmc2209::reg::IHOLD_IRUN::default();
        // let mut rampmode = tmc2209::reg::RAMPMODE::default();
        // let mut vmax = tmc2209::reg::VMAX::default();
        // let mut amax = tmc2209::reg::AMAX::default();
        let mut tpwmthrs = tmc2209::reg::TPWMTHRS::default();
        let mut chopconf = tmc2209::reg::CHOPCONF::default();

        ihold_run.set_ihold(ihold);
        ihold_run.set_irun(irun);
        ihold_run.set_ihold_delay(1);
        // rampmode.set(1);
        // vmax.set(20000);
        // amax.set(1000);
        tpwmthrs.set(500);
        chopconf.set_toff(5);
        chopconf.set_hend(1);
        chopconf.set_hstrt(4);
        chopconf.set_tbl(2);
        chopconf.set_mres(0);
        // chopconf.set_chm(false);

        let addr = self.tmc_addr;
        tmc2209::send_write_request(addr, gconf, &mut self.uart_tx, &mut self.uart_rx)
            .map_err(|_| AtomiError::DtuTMCWriteRequestError)?;
        tmc2209::send_write_request(addr, clear_gstat, &mut self.uart_tx, &mut self.uart_rx)
            .map_err(|_| AtomiError::DtuTMCWriteRequestError)?;
        tmc2209::send_write_request(addr, chopconf, &mut self.uart_tx, &mut self.uart_rx)
            .map_err(|_| AtomiError::DtuTMCWriteRequestError)?;
        tmc2209::send_write_request(addr, ihold_run, &mut self.uart_tx, &mut self.uart_rx)
            .map_err(|_| AtomiError::DtuTMCWriteRequestError)?;
        if let Some(sgthres) = sgthres {
            tmc2209::send_write_request(addr, sgthres, &mut self.uart_tx, &mut self.uart_rx)
                .map_err(|_| AtomiError::DtuTMCWriteRequestError)?;
        }
        Ok(())
    }
    //
    pub fn enable(&mut self) -> Result<(), AtomiError> {
        // info!("tmc driver enable");
        self.enable_pin.set_low().map_err(|_| AtomiError::GpioPinError)
    }

    pub fn disable(&mut self) -> Result<(), AtomiError> {
        // info!("tmc driver disable");
        self.enable_pin.set_high().map_err(|_| AtomiError::GpioPinError)
    }

    pub fn ensure_enable(&mut self) -> Result<(), AtomiError> {
        if self.enable_pin.is_set_high().map_err(|_| AtomiError::GpioPinError)? {
            self.enable()
        } else {
            Ok(())
        }
    }

    pub fn set_direction(&mut self, forward: bool) -> Result<(), AtomiError> {
        if forward ^ self.revert_dir {
            self.dir_pin.set_high().map_err(|_| AtomiError::GpioPinError)?;
        } else {
            self.dir_pin.set_low().map_err(|_| AtomiError::GpioPinError)?;
        }
        Ok(())
    }

    pub fn set_speed(&mut self, speed: u32) {
        self.speed = speed;
        if self.speed != 0 {
            self.wait_period = (1_000_000 / (speed * 2)) as u64;
        }
    }

    pub async fn step(&mut self) -> Result<(), AtomiError> {
        if self.speed == 0 {
            return Err(AtomiError::MmdMoveWithZeroSpeed);
        }
        self.step_pin.set_high().map_err(|_| AtomiError::GpioPinError)?;
        self.async_delay.delay(self.wait_period.micros()).await;
        self.step_pin.set_low().map_err(|_| AtomiError::GpioPinError)?;
        self.async_delay.delay(self.wait_period.micros()).await;
        Ok(())
    }

    // pub fn sg_result(&mut self) -> Result<tmc2209::reg::SG_RESULT, AtomiError> {
    //     let reader = tmc2209::Reader::default();
    //     Ok(self
    //         .read_reg::<tmc2209::reg::SG_RESULT>(&mut self.uart_tx, &mut self.uart_rx, &mut reader)
    //         .expect("read failed"))
    // }

    // todo(zephyr): 现在还不需要speed mode，暂时删掉下面代码。
    // // TODO:set_speed with acceleration, need test
    // pub fn set_speed(&mut self, speed: u32, accl: u32) {
    //     self.target_speed = speed;
    //     self.current_speed = 0;
    //     self.acceleration = accl;
    // }
    //
    // pub fn update_current_speed(&mut self) {
    //     if self.current_speed < self.target_speed {
    //         self.current_speed = min(self.current_speed + self.acceleration, self.target_speed);
    //         if self.current_speed > 0 {
    //             self.wait_period = (1_000_000 / (self.current_speed * 2)) as u64;
    //         }
    //     }
    // }
    //
    // pub async fn step(&mut self) -> Result<(), AtomiError> {
    //     self.update_current_speed();
    //     if self.speed == 0 {
    //         return Err(AtomiError::MmdMoveWithZeroSpeed);
    //     }
    //     self.step_pin.set_high().map_err(|_| AtomiError::GpioPinError)?;
    //     self.async_delay.delay(self.wait_period.micros()).await;
    //     self.step_pin.set_low().map_err(|_| AtomiError::GpioPinError)?;
    //     self.async_delay.delay(self.wait_period.micros()).await;
    //     Ok(())
    // }

    // pub async fn run(&mut self, steps: u32, speed: u32) -> Result<u32, T::Error> {
    //     self.should_stop.store(false, Ordering::Relaxed);
    //
    //     let wait_period = 1_000_000 / (speed * 2) as u64;
    //     for i in 0..steps {
    //         if self.should_stop.load(Ordering::Relaxed) {
    //             return Ok(i);
    //         }
    //         self.step_pin.set_high()?;
    //         self.async_delay.delay(wait_period.micros()).await;
    //         self.step_pin.set_low()?;
    //         self.async_delay.delay(wait_period.micros()).await;
    //     }
    //     Ok(steps)
    // }
    //
    // pub fn stop(&mut self) {
    //     self.should_stop.store(true, Ordering::Relaxed)
    // }

    // fn read_reg<R: ReadableRegister>(
    //     tx: &mut U,
    //     rx: &mut X,
    //     tmc_reader: &mut tmc2209::Reader,
    // ) -> Option<R> {
    //     let mut buffer = [0u8; 32];
    //     tmc2209::send_read_request::<R, _, _>(0, tx, rx).unwrap();

    //     let mut timeout = 50000u64;
    //     loop {
    //         match rx.read_raw(&mut buffer) {
    //             Ok(bytes) => {
    //                 // info!("bytes:{}, buffer:{}", bytes, buffer);
    //                 timeout = 50000u64;
    //                 let (_processed_bytes, tmc_response) =
    //                     tmc_reader.read_response(&buffer[..bytes]);
    //                 // info!(
    //                 //     "processed bytes: {}, recv bytes: {}",
    //                 //     processed_bytes, bytes
    //                 // );
    //                 let res = if let Some(response) = tmc_response {
    //                     if !response.crc_is_valid() {
    //                         error!("Received invalid response!");
    //                         return None;
    //                     }
    //                     // info!("Received valid response: {}", Debug2Format(&response));
    //                     match response.reg_addr() {
    //                         Ok(addr) => {
    //                             // info!("xfguo: addr = {}, R::ADDR = {}", Debug2Format(&addr), Debug2Format(&R::ADDRESS));
    //                             if addr == R::ADDRESS {
    //                                 let reg = response.register::<R>().unwrap();
    //                                 // info!("xfguo: {}: {}", Debug2Format(&R::ADDRESS), Debug2Format(&reg));
    //                                 Some(reg)
    //                             } else {
    //                                 None
    //                             }
    //                         }
    //                         _ => None,
    //                     }
    //                 } else {
    //                     None
    //                 };
    //                 if res.is_none() {
    //                     continue;
    //                 }
    //                 return res;
    //             }
    //             Err(nb::Error::WouldBlock) => {
    //                 if timeout > 0 {
    //                     timeout -= 1;
    //                     delay(10);
    //                     continue;
    //                 } else {
    //                     error!("timeout in reading");
    //                     return None;
    //                 }
    //             }
    //             Err(_) => {
    //                 error!("Errors in read data");
    //                 return None;
    //             }
    //         }
    //     }
    // }
}

// type UartPinsType = (Pin<Gpio8, FunctionUart, PullUp>, Pin<Gpio9, FunctionUart, PullUp>);