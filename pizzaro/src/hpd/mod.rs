use core::sync::atomic::AtomicBool;
use rp2040_hal::gpio::bank0::{Gpio8, Gpio9};
use rp2040_hal::gpio::{FunctionUart, Pin, PullDown};
use rp2040_hal::pac::UART1;
use rp2040_hal::uart::{Enabled, UartPeripheral};

pub static GLOBAL_LINEAR_BULL_STOP: AtomicBool = AtomicBool::new(false);
pub type HpdUartType = UartPeripheral<
    Enabled,
    UART1,
    (
        Pin<Gpio8, FunctionUart, PullDown>,
        Pin<Gpio9, FunctionUart, PullDown>,
    ),
>;

pub mod hpd_misc;
pub mod linear_bull_processor;
pub mod linear_scale;
pub mod pid;
