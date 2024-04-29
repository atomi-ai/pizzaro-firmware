/// BSP version: Spring_Begins
///
/// TODO(zephyr): Add schematic pdf below.
///
/// DTU Schematic: [[url]]
use crate::define_pins;
use rp2040_hal::gpio::bank0::{Gpio0, Gpio1, Gpio18};
use rp2040_hal::gpio::{FunctionSioOutput, FunctionUart, Pin, PullDown, PullUp};
use rp2040_hal::pac::{Interrupt, UART0};
use rp2040_hal::uart::{Enabled, UartPeripheral};

pub type DtuUartPins = (Pin<Gpio0, FunctionUart, PullDown>, Pin<Gpio1, FunctionUart, PullDown>);
pub type DtuUartType = UartPeripheral<Enabled, UART0, DtuUartPins>;
pub type DtuUartDirPinType = Pin<Gpio18, FunctionSioOutput, PullUp>;

define_pins! {
    dtu_uart, UART0
}

pub fn dtu_uart_irq() -> Interrupt {
    Interrupt::UART0_IRQ
}

define_pins! {
    // DTU: Detacher Unit
    dtu_sys_tx, gpio0,
    dtu_sys_rx, gpio1,
    dtu_485_dir, gpio18,
    dtu_stepper_step, gpio2,
    dtu_stepper_dir, gpio3,
    dtu_stepper_nEN, gpio24,
    dtu_stepper_diag, gpio19,
    dtu_limit0, gpio26,
    dtu_limit1, gpio27,
    dtu_tmc_uart_tx, gpio8,
    dtu_tmc_uart_rx, gpio9
}
