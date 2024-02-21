use crate::global_timer::{now, Delay};

use cortex_m::prelude::_embedded_hal_spi_FullDuplex;
use defmt::{info, warn};
use fugit::ExtU64;
use rp2040_hal::gpio::bank0::{Gpio12, Gpio14, Gpio15};
use rp2040_hal::gpio::{FunctionSpi, Pin, PullNone, PullUp};
use rp2040_hal::pac::SPI1;
use rp2040_hal::spi::Enabled;
use rp2040_hal::Spi;

const MIN_TIMESTAMP_GAP_FOR_NEXT_DATA: u64 = 50_000;

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct CdpKeepResult {
    // TODO(zephyr): change to Instant
    timestamp: u64,
    position: i32,
    speed: f64, // unit mm/s
}

impl CdpKeepResult {
    pub fn export(&self) -> (u64, i32) {
        (self.timestamp, self.position)
    }
}

pub struct CapDisplacementProcessor {
    buffer: [u8; 3],
    count: usize,
    last_timestamp: u64,

    result: Option<CdpKeepResult>,
}

impl Default for CapDisplacementProcessor {
    fn default() -> Self {
        Self::new()
    }
}

impl CapDisplacementProcessor {
    pub fn new() -> Self {
        CapDisplacementProcessor {
            buffer: [0; 3],
            count: 0,
            last_timestamp: 0,
            result: None,
        }
    }

    pub fn get_last_timestamp(&self) -> u64 {
        self.last_timestamp
    }

    pub fn process(&mut self, data: u8, timestamp: u64) -> Option<i32> {
        // info!(
        //     "in cap process, timestamp:{}, last_timestamp:{}, if cond:{}, count:{}, data:{}",
        //     timestamp,
        //     self.last_timestamp,
        //     timestamp < self.last_timestamp + MIN_TIMESTAMP_GAP_FOR_NEXT_DATA,
        //     self.count,
        //     data
        // );
        let res = if timestamp < self.last_timestamp + MIN_TIMESTAMP_GAP_FOR_NEXT_DATA {
            // TODO(zephyr): 这个有100ms延迟，我们可以考虑用更好的方法来替代。
            if self.count == 3 {
                self.buffer[0] = self.buffer[1];
                self.buffer[1] = self.buffer[2];
                self.count = 2;
            }
            None
        } else {
            let res = if self.count == 3 {
                self.parse_data()
            } else {
                None
            };
            self.count = 0;
            // info!("res:{} ", res);
            if let Some(data) = res {
                let speed = match self.result {
                    None => 0.0,
                    Some(last_res) => {
                        ((data - last_res.position) as f64) * 1000.0
                            / ((self.last_timestamp - last_res.timestamp) as f64)
                    }
                };
                self.result = Some(CdpKeepResult {
                    timestamp: self.last_timestamp,
                    position: data,
                    speed,
                });
                // info!("result update:{}", self.result);
            }
            res
        };
        // info!("set buffer[{}] = {}", self.count, data);
        self.buffer[self.count] = data;
        self.count += 1;
        self.last_timestamp = timestamp;
        res
    }

    pub fn emit_result(&self) -> Option<CdpKeepResult> {
        self.result
    }

    fn parse_data(&self) -> Option<i32> {
        if self.buffer[2] & 0b11100000 != 0 {
            return None;
        }
        let mut res = (((self.buffer[2] as i32) & 0xf) << 16)
            | ((self.buffer[1] as i32) << 8)
            | self.buffer[0] as i32;
        if self.buffer[2] & 0x10 != 0 {
            res *= -1;
        }
        Some(res)
    }
}

pub struct CdpAdapter(&'static mut CapDisplacementProcessor);

impl CdpAdapter {
    pub fn new(cdp: &'static mut CapDisplacementProcessor) -> Self {
        Self(cdp)
    }

    pub fn process(&mut self, data: u8, timestamp: u64) -> Option<i32> {
        critical_section::with(|_cs| self.0.process(data, timestamp))
    }

    pub fn get_last_timestamp(&mut self) -> u64 {
        critical_section::with(|_cs| self.0.get_last_timestamp())
    }

    pub fn emit_result(&self) -> Option<CdpKeepResult> {
        critical_section::with(|_cs| self.0.emit_result())
    }
}

pub type CapDisplacementSpiType = Spi<
    Enabled,
    SPI1,
    (
        Pin<Gpio15, FunctionSpi, PullUp>,
        Pin<Gpio12, FunctionSpi, PullNone>,
        Pin<Gpio14, FunctionSpi, PullNone>,
    ),
    8,
>;

pub async fn monitor_cap_displacement_spi(mut spi: CapDisplacementSpiType, mut cdp: CdpAdapter) {
    loop {
        let data = spi.read();
        let _ = spi.send(0);

        let last = cdp.get_last_timestamp();
        let ts = now().ticks();
        if ts - last > 1_000_000 {
            warn!("CAP DISPLACEMENT READ ERROR");
        }

        #[allow(clippy::single_match)]
        match data {
            Ok(d) => {
                let ts = now().ticks();
                let _res = cdp.process(d.reverse_bits(), ts);
                // info!("monitor_cap_displacement() 3.5: data = {}, ts = {}", d, ts);
            }
            _ => {} // ignore on errors
        }
        Delay::new(1.millis()).await;
    }
}

pub async fn read_cdp_result(cdp: CdpAdapter) {
    loop {
        let res = cdp.emit_result();
        info!("read_cdp_result() 5: got CDP result: {:?}", res);
        Delay::new(100.millis()).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_data_within_short_intervals() {
        let mut processor = CapDisplacementProcessor::new();
        // 短时间间隔内接收数据
        assert_eq!(processor.process(0x01, 100_000), None);
        assert_eq!(processor.process(0x02, 100_020), None);
        assert_eq!(processor.process(0x03, 100_040), None);
        // 尚未生成结果
        assert_eq!(processor.count, 3);
    }

    #[test]
    fn test_data_within_long_intervals() {
        let mut processor = CapDisplacementProcessor::new();
        // 长时间间隔内接收数据
        assert_eq!(processor.process(0x01, 100_000), None);
        assert_eq!(processor.process(0x02, 101_000), None);
        assert_eq!(processor.process(0x03, 102_000), None);
        assert_eq!(processor.count, 3);
        assert_eq!(processor.process(0x04, 200_000), Some(0x030201));
        assert_eq!(processor.count, 1);
    }

    #[test]
    fn test_data_with_mixed_intervals() {
        let mut processor = CapDisplacementProcessor::new();
        // 不规则时间间隔
        assert_eq!(processor.process(0x01, 100_000), None);
        assert_eq!(processor.process(0x02, 100_020), None);
        assert_eq!(processor.process(0x03, 101_000), None);
        assert_eq!(processor.process(0x04, 151_020), Some(0x030201));
        assert_eq!(processor.process(0x05, 150_040), None);
        assert_eq!(processor.process(0x06, 151_040), None);
        assert_eq!(processor.process(0x07, 300_000), Some(0x060504));
        assert_eq!(processor.process(0x08, 301_000), None);
        assert_eq!(processor.process(0x99, 302_000), None);
        assert_eq!(processor.process(0x07, 400_000), None); // 0b11100000检查错误, clear buf
        assert_eq!(processor.process(0x0A, 400_000), None);
        assert_eq!(processor.process(0x1B, 400_000), None);
        assert_eq!(processor.process(0x01, 500_000), Some(-0xB0A07));
        assert_eq!(processor.process(0x02, 500_000), None);
        assert_eq!(processor.process(0x03, 500_000), None);
        assert_eq!(processor.process(0x00, 500_000), None); // overflow
        assert_eq!(processor.count, 3);
        assert_eq!(processor.process(0x00, 600_000), Some(0x000302));
        assert_eq!(processor.count, 1);
    }
}
