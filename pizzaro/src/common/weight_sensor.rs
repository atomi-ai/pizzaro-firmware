use crate::common::global_timer::Delay;
use bsp::hal::gpio::{DynPinId, FunctionSio, Pin, PullDown, PullUp, SioInput, SioOutput};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use fugit::ExtU64;
use rp_pico as bsp;

struct Hx711 {
    sck: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
    dout: Pin<DynPinId, FunctionSio<SioInput>, PullUp>,
}

impl Hx711 {
    pub fn new(
        sck: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
        dout: Pin<DynPinId, FunctionSio<SioInput>, PullUp>,
    ) -> Self {
        Self { sck, dout }
    }

    pub async fn read_weight(&mut self) -> u32 {
        let mut result: u32 = 0;

        // 设置时钟为低
        self.sck.set_low().unwrap();

        // 等待数据准备好
        while self.dout.is_high().unwrap() {}
        for _ in 0..25 {
            // 设置时钟为高\
            self.sck.set_high().unwrap();
            Delay::new(1.micros()).await; // 延时1微秒

            // 读取数据
            result <<= 1;
            if self.dout.is_high().unwrap() {
                result |= 1;
            }

            // 设置时钟为低
            self.sck.set_low().unwrap();
            Delay::new(1.micros()).await; // 延时1微秒
        }

        result
    }
}

pub struct WeightSensors {
    sensor1: Hx711,
    sensor2: Hx711,
    sensor3: Hx711,
    sensor4: Hx711,
}

impl WeightSensors {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        sck1: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
        dout1: Pin<DynPinId, FunctionSio<SioInput>, PullUp>,
        sck2: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
        dout2: Pin<DynPinId, FunctionSio<SioInput>, PullUp>,
        sck3: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
        dout3: Pin<DynPinId, FunctionSio<SioInput>, PullUp>,
        sck4: Pin<DynPinId, FunctionSio<SioOutput>, PullDown>,
        dout4: Pin<DynPinId, FunctionSio<SioInput>, PullUp>,
    ) -> Self {
        Self {
            sensor1: Hx711::new(sck1, dout1),
            sensor2: Hx711::new(sck2, dout2),
            sensor3: Hx711::new(sck3, dout3),
            sensor4: Hx711::new(sck4, dout4),
        }
    }

    pub async fn get_all_weights(&mut self) -> (u32, u32, u32, u32) {
        (
            self.sensor1.read_weight().await,
            self.sensor2.read_weight().await,
            self.sensor3.read_weight().await,
            self.sensor4.read_weight().await,
        )
    }

    // 操作所有传感器的方法
}
