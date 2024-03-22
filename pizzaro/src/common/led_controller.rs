use fugit::ExtU64;
use rp2040_hal::gpio::AnyPin;
use rp2040_hal::pio::StateMachineIndex;
use rp_pico::hal::pio::PIOExt;
use smart_leds::RGB8;
use smart_leds_trait::SmartLedsWrite;
use ws2812_pio::Ws2812Direct;

use super::global_timer::Delay;

enum LedState {
    On,
    Off,
}

pub trait LedController {
    fn set_color(&mut self, color: RGB8);
}

impl<P, SM, I> LedController for Ws2812Direct<P, SM, I>
where
    I: AnyPin<Function = P::PinFunction>,
    P: PIOExt,
    SM: StateMachineIndex,
{
    fn set_color(&mut self, color: RGB8) {
        // SmartLedsWrite::write(&mut self, [color].iter().copied()).unwrap();
        self.write([color].iter().copied()).unwrap();
    }
}

pub struct MyLED<T: LedController> {
    led_controller: T,
    ledon: RGB8,
    ledoff: RGB8,
    state: LedState,
}

impl<T: LedController> MyLED<T> {
    pub fn new(ledon: RGB8, ledoff: RGB8, led_controller: T) -> Self {
        Self { ledoff, ledon, led_controller, state: LedState::Off }
    }

    pub fn ledon(&mut self) {
        self.state = LedState::On;
        self.flush();
        // defmt::info!("ledon")
    }

    pub fn ledoff(&mut self) {
        self.state = LedState::Off;
        self.flush();
        // defmt::info!("ledoff")
    }

    fn flush(&mut self) {
        match self.state {
            LedState::Off => self.led_controller.set_color(self.ledoff),
            LedState::On => self.led_controller.set_color(self.ledon),
        };
    }

    pub fn toggle(&mut self) {
        self.state = match self.state {
            LedState::Off => LedState::On,
            LedState::On => LedState::Off,
        };
        self.flush();
    }
}

pub struct MyLedAdapter<T: LedController + 'static>(&'static mut MyLED<T>);

impl<T: LedController> MyLedAdapter<T> {
    pub fn new(led: &'static mut MyLED<T>) -> Self {
        Self(led)
    }

    pub fn ledon(&mut self) {
        critical_section::with(|_cs| self.0.ledon())
    }

    pub fn ledoff(&mut self) {
        critical_section::with(|_cs| self.0.ledoff())
    }

    // fn flush(&mut self) {
    //     critical_section::with(|_cs| self.0.flush())
    // }

    pub fn toggle(&mut self) {
        critical_section::with(|_cs| self.0.toggle())
    }
}

pub async fn blinky_smart_led<T: LedController>(mut led: MyLED<T>) {
    loop {
        led.ledoff();
        Delay::new(500.millis()).await;
        led.ledon();
        Delay::new(500.millis()).await;
        // info!("led toggle");
    }
}
