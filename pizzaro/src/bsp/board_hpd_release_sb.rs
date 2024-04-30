/// BSP version: Spring_Begins
///
/// HPD Schematic: [[https://github.com/atomi-ai/pizzaro-v1.5/blob/master/hardware/HPD-v1_5_4.pdf]]
use crate::define_pins;
use rp2040_hal::gpio::bank0::{Gpio12, Gpio8, Gpio9};
use rp2040_hal::gpio::{FunctionSioOutput, FunctionUart, Pin, PullDown, PullUp};
use rp2040_hal::pac::{Interrupt, UART1};
use rp2040_hal::uart::{Enabled, UartPeripheral};

pub type HpdUartPins = (Pin<Gpio8, FunctionUart, PullDown>, Pin<Gpio9, FunctionUart, PullDown>);
pub type HpdUartType = UartPeripheral<Enabled, UART1, HpdUartPins>;
pub type HpdUartDirPinType = Pin<Gpio12, FunctionSioOutput, PullUp>;

define_pins! {
    hpd_uart, UART1
}

pub fn hpd_uart_irq() -> Interrupt {
    Interrupt::UART1_IRQ
}

define_pins! {
    // HPD: HighPowerMotorDriver
    // 150w brush motor
    hpd_br_pwm_a, gpio0,
    hpd_br_pwm_b, gpio1,
    hpd_br_nEN, gpio2,
    // PWM fan
    hpd_fan_en, gpio3,
    hpd_fan_speed_sense, gpio4,
    hpd_fan_speed_ctrl, gpio6,
    // 485 interface
    hpd_sys_tx, gpio8,
    hpd_sys_rx, gpio9,
    hpd_485_dir, gpio12,
    // optical linear scale
    hpd_opt_a, gpio10,
    hpd_opt_b, gpio11,
    // safety barrier
    hpd_safety_barrier, gpio14
}

// define pwm slices
define_pins! {
    hpd_motor_pwm_slice, pwm0
}

// define pwm channel
define_pins! {
    hpd_motor_pwm_a_channel, channel_a,
    hpd_motor_pwm_b_channel, channel_b
}
