/// BSP version: Spring_Begins
///
/// TODO(lv): Please check the correctness.
///
/// MC Schematic: [[https://github.com/atomi-ai/pizzaro-v1.5/blob/master/hardware/MC-v1_5_4.pdf]]
use crate::define_pins;
use rp2040_hal::gpio::bank0::{Gpio11, Gpio12, Gpio13, Gpio8, Gpio9};
use rp2040_hal::gpio::{FunctionSioOutput, FunctionUart, Pin, PullDown, PullUp};
use rp2040_hal::pac::{Interrupt, UART0, UART1};
use rp2040_hal::uart::{Enabled, UartPeripheral};

pub type McUartPins = (Pin<Gpio12, FunctionUart, PullDown>, Pin<Gpio13, FunctionUart, PullDown>);
pub type McUartType = UartPeripheral<Enabled, UART0, McUartPins>;
pub type McUartDirPinType = Pin<Gpio11, FunctionSioOutput, PullUp>;

pub type McUiScreenPins = (Pin<Gpio8, FunctionUart, PullDown>, Pin<Gpio9, FunctionUart, PullDown>);
pub type McUiUartType = UartPeripheral<Enabled, UART1, McUiScreenPins>;

define_pins! {
    mc_uart, UART0,
    mc_ui_uart, UART1
}

pub fn mc_ui_uart_irq() -> Interrupt {
    Interrupt::UART1_IRQ
}

define_pins! {
    // MC: MainController
    mc_cap_sck0, gpio0,
    mc_cap_dout0, gpio1,
    mc_cap_sck1, gpio2,
    mc_cap_dout1, gpio3,
    mc_cap_sck2, gpio4,
    mc_cap_dout2, gpio5,
    mc_cap_sck3, gpio6,
    mc_cap_dout3, gpio7,
    mc_ui_uart_tx, gpio8,
    mc_ui_uart_rx, gpio9,
    mc_485_dir, gpio11,
    mc_sys_tx, gpio12,
    mc_sys_rx, gpio13,
    mc_emstop, gpio14,
    mc_emled, gpio15
}
