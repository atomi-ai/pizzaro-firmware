use cortex_m::asm::delay;
use embedded_hal::digital::OutputPin;
use smart_leds_trait::{SmartLedsWrite, RGB8};

pub struct Ws2812<P: OutputPin> {
    pin: P,
}

impl<P: OutputPin> Ws2812<P> {
    /// The timer has to already run at with a frequency of 3 MHz
    pub fn new(mut pin: P) -> Ws2812<P> {
        pin.set_low().ok();
        Self { pin }
    }

    fn write_byte(&mut self, mut data: u8) {
        for _ in 0..8 {
            if (data & 0x80) != 0 {
                // self.delay(1);
                delay(40);
                self.pin.set_high().ok();
                // self.delay(2);
                delay(60);
                self.pin.set_low().ok();
            } else {
                // self.delay(1);
                delay(50);
                self.pin.set_high().ok();
                delay(5);
                self.pin.set_low().ok();
                delay(40);
                // self.delay(2);
            }
            data <<= 1;
        }
    }

    fn delay(&mut self, cycle: u16) {
        for _ in 0..cycle {
            delay(50);
        }
    }
}

impl<P: OutputPin> SmartLedsWrite for Ws2812<P> {
    type Error = ();
    type Color = RGB8;
    /// Write all the items of an iterator to a ws2812 strip
    fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: Iterator<Item = I>,
        I: Into<Self::Color>,
    {
        for item in iterator {
            let item = item.into();
            self.write_byte(item.g);
            self.write_byte(item.r);
            self.write_byte(item.b);
        }
        // Get a timeout period of 300 ns
        self.delay(900);
        Ok(())
    }
}
