/// BSP version: Spring_Begins
///
/// TODO(zephyr): Add schematic pdf below.
///
/// ASD (Adaptive Scooping Device) Schematic: [[url]]
use crate::define_pins;
use rp2040_hal::gpio::bank0::{Gpio17, Gpio20, Gpio21};
use rp2040_hal::gpio::{FunctionSioOutput, FunctionUart, Pin, PullDown, PullUp};
use rp2040_hal::pac::{Interrupt, UART1};
use rp2040_hal::uart::{Enabled, UartPeripheral};

// 18 - tx / 19 - rx
pub type AsdUartPins = (Pin<Gpio20, FunctionUart, PullDown>, Pin<Gpio21, FunctionUart, PullDown>);
pub type AsdUartType = UartPeripheral<Enabled, UART1, AsdUartPins>;
pub type AsdUartDirPinType = Pin<Gpio17, FunctionSioOutput, PullUp>;

define_pins! {
    asd_uart, UART1
}

pub fn asd_uart_irq() -> Interrupt {
    Interrupt::UART1_IRQ
}

define_pins! {
    asd_sys_tx, gpio20,
    asd_sys_rx, gpio21,
    asd_485_dir, gpio17,  // unused
    asd_stepper_step, gpio7,
    asd_stepper_dir, gpio6,
    asd_stepper_n_en, gpio8
}
