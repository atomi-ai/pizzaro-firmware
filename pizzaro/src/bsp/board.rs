// BSP version: 1.5.4
use rp2040_hal::{
    gpio::{
        bank0::{
            Gpio11, Gpio12, Gpio13, Gpio17, Gpio18, Gpio19, Gpio21, Gpio22, Gpio23, Gpio24, Gpio25,
            Gpio26, Gpio27, Gpio4, Gpio5, Gpio8, Gpio9,
        },
        FunctionSio, FunctionSioOutput, FunctionUart, Pin, PullDown, PullUp, SioInput, SioOutput,
    },
    pac::{Interrupt, UART0, UART1},
    pwm::{Pwm0, Pwm4, Pwm5},
    uart::{Enabled, UartPeripheral},
};

use crate::{
    common::global_timer::DelayCreator,
    mmd::{linear_stepper::LinearStepper, pwm_motor::PwmMotor, stepper::Stepper},
};

macro_rules! define_pins {
    ($($alias:ident, $pin:tt),*) => {
        $(
	    #[macro_export]
            macro_rules! $alias {
                ($pins:expr) => {
                    $pins.$pin
                };
            }
        )*
    };
}

pub type HpdUartPins = (
    Pin<Gpio8, FunctionUart, PullDown>,
    Pin<Gpio9, FunctionUart, PullDown>,
);
pub type HpdUartType = UartPeripheral<Enabled, UART1, HpdUartPins>;
pub type HpdUartDirPinType = Pin<Gpio12, FunctionSioOutput, PullUp>;

pub type MmdUartPins = (
    Pin<Gpio4, FunctionUart, PullDown>,
    Pin<Gpio5, FunctionUart, PullDown>,
);
pub type MmdUartType = UartPeripheral<Enabled, UART1, MmdUartPins>;
pub type MmdUartDirPinType = Pin<Gpio21, FunctionSioOutput, PullUp>;

pub type McUartPins = (
    Pin<Gpio12, FunctionUart, PullDown>,
    Pin<Gpio13, FunctionUart, PullDown>,
);
pub type McUartType = UartPeripheral<Enabled, UART0, McUartPins>;
pub type McUartDirPinType = Pin<Gpio11, FunctionSioOutput, PullUp>;

pub type McUiScreenPins = (
    Pin<Gpio8, FunctionUart, PullDown>,
    Pin<Gpio9, FunctionUart, PullDown>,
);
pub type McUiUartType = UartPeripheral<Enabled, UART1, McUiScreenPins>;

// 42 motor0(and check pins defined in macros)
pub type MmdStepper42_0EnablePinType = Pin<Gpio22, FunctionSio<SioOutput>, PullDown>;
pub type MmdStepper42_0DirPinType = Pin<Gpio18, FunctionSio<SioOutput>, PullDown>;
pub type MmdStepper42_0StepPinType = Pin<Gpio11, FunctionSio<SioOutput>, PullDown>;
pub type ConveyorBeltRotationMotorType = Stepper<
    MmdStepper42_0EnablePinType,
    MmdStepper42_0DirPinType,
    MmdStepper42_0StepPinType,
    DelayCreator,
>;

// 42 motor1(and check pins defined in macros)
pub type MmdStepper42_1EnablePinType = Pin<Gpio25, FunctionSio<SioOutput>, PullDown>;
pub type MmdStepper42_1DirPinType = Pin<Gpio23, FunctionSio<SioOutput>, PullDown>;
pub type MmdStepper42_1StepPinType = Pin<Gpio19, FunctionSio<SioOutput>, PullDown>;
pub type MmdLimitSwitch0 = Pin<Gpio26, FunctionSio<SioInput>, PullDown>;
pub type MmdLimitSwitch1 = Pin<Gpio27, FunctionSio<SioInput>, PullDown>;
pub type ConveyorBeltLinearStepperType = Stepper<
    MmdStepper42_1EnablePinType,
    MmdStepper42_1DirPinType,
    MmdStepper42_1StepPinType,
    DelayCreator,
>;
pub type ConveyorBeltLinearBullType = LinearStepper<
    MmdLimitSwitch0,
    MmdLimitSwitch1,
    MmdStepper42_1EnablePinType,
    MmdStepper42_1DirPinType,
    MmdStepper42_1StepPinType,
    DelayCreator,
>;

// 57 motor(and check pins defined in macros)
pub type MmdStepper57EnablePinType = Pin<Gpio24, FunctionSio<SioOutput>, PullDown>;
pub type MmdStepper57DirPinType = Pin<Gpio17, FunctionSio<SioOutput>, PullDown>;
pub type MmdStepper57StepPinType = Pin<Gpio13, FunctionSio<SioOutput>, PullDown>;
pub type PresserRotationMotorType = Stepper<
    MmdStepper57EnablePinType,
    MmdStepper57DirPinType,
    MmdStepper57StepPinType,
    DelayCreator,
>;

pub type MmdDisperser0MotorType = PwmMotor<Pwm4>;
pub type MmdDisperser1MotorType = PwmMotor<Pwm5>;
pub type MmdPeristalicPumpMotorType = PwmMotor<Pwm0>;

define_pins! {
    mc_uart, UART0,
    mc_ui_uart, UART1,
    mmd_uart, UART1,
    hpd_uart, UART1
}

pub fn hpd_uart_irq() -> Interrupt {
    Interrupt::UART1_IRQ
}

pub fn mmd_uart_irq() -> Interrupt {
    Interrupt::UART1_IRQ
}

pub fn mc_ui_uart_irq() -> Interrupt {
    Interrupt::UART1_IRQ
}

define_pins! {
    // ws2812b led on mc/mmd/hpd
    smart_led, gpio16,

    // // V1.5.4
    // // MMD: MultiMotorDriver
    // // brush motor
    // mmd_br_pwm_a, gpio0,
    // mmd_br_pwm_b, gpio1,
    // mmd_br_nEN, gpio3,
    // // 485 interface
    // mmd_sys_tx, gpio4,
    // mmd_sys_rx, gpio5,
    // mmd_485_dir, gpio8,
    // // brushless motor0
    // mmd_spd_sense_bl0, gpio2,
    // mmd_dir_bl0, gpio20,
    // mmd_spd_ctrl_bl0, gpio9,
    // // brushless motor1
    // mmd_spd_sense_bl1, gpio6,
    // mmd_dir_bl1, gpio7,
    // mmd_spd_ctrl_bl1, gpio10,
    // // 42 motor0
    // mmd_stepper42_step0, gpio11,
    // mmd_stepper42_dir0, gpio18,
    // mmd_stepper42_nEN0, gpio22,
    // mmd_stepper42_diag0, gpio28,
    // // 42 motor1
    // mmd_stepper42_step1, gpio21,
    // mmd_stepper42_dir1, gpio23,
    // mmd_stepper42_nEN1, gpio25,
    // mmd_stepper42_diag1, gpio29,
    // // 57 motor
    // mmd_stepper57_nEN, gpio24,
    // mmd_stepper57_step, gpio14,
    // mmd_stepper57_dir, gpio17,
    // // tmc related interface
    // mmd_tmc_uart_tx, gpio12,
    // mmd_tmc_uart_rx, gpio13,
    // mmd_tmc_5160_addr, gpio29,
    // // proximity sensors
    // mmd_proximity_sensor0, gpio15,
    // mmd_proximity_sensor1, gpio19,
    // // limit switchs
    // mmd_limit0, gpio26,
    // mmd_limit1, gpio27,

    // rollback to HW version V1.5.1
    // MMD: MultiMotorDriver
    // brush motor
    mmd_br_pwm_a, gpio0,
    mmd_br_pwm_b, gpio1,
    mmd_br_nEN, gpio3,
    // 485 interface
    mmd_sys_tx, gpio4,
    mmd_sys_rx, gpio5,
    mmd_485_dir, gpio21,
    // brushless motor0
    mmd_spd_sense_bl0, gpio2,
    mmd_dir_bl0, gpio20,
    mmd_spd_ctrl_bl0, gpio9,
    // brushless motor1
    mmd_spd_sense_bl1, gpio6,
    mmd_dir_bl1, gpio7,
    mmd_spd_ctrl_bl1, gpio10,
    // 42 motor0
    mmd_stepper42_step0, gpio11, // PWM5
    mmd_stepper42_dir0, gpio18,
    mmd_stepper42_nEN0, gpio22,
    mmd_stepper42_diag0, gpio28,
    // 42 motor1
    mmd_stepper42_step1, gpio19,
    mmd_stepper42_dir1, gpio23,
    mmd_stepper42_nEN1, gpio25,	// not exists, in v1.5.1, 42_nEN0/1 are connected to gpio22
    mmd_stepper42_diag1, gpio29,
    // 57 motor
    mmd_stepper57_nEN, gpio24,
    mmd_stepper57_step, gpio13,
    mmd_stepper57_dir, gpio17,
    // tmc related interface
    mmd_tmc_uart_tx, gpio12,
    //mmd_tmc_uart_rx, gpio13,	// not exists
    //mmd_tmc_5160_addr, gpio29,	// not exists
    // proximity sensors
    //mmd_proximity_sensor0, gpio15, // not exists
    //mmd_proximity_sensor1, gpio19, // not exists
    // limit switchs
    mmd_limit0, gpio26,
    mmd_limit1, gpio27,


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
    hpd_safety_barrier, gpio14,

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
    mc_emled, gpio15,

    demo_pwm_a, gpio0,
    demo_pwm_b, gpio1,
    demo_pwm_en, gpio2,
    demo_ols_a, gpio10,
    demo_ols_b, gpio11
}

// define pwm slices
define_pins! {
    mmd_motor42_pwm_slice0, pwm5,
    mmd_motor42_pwm_slice1, pwm2,
    mmd_stepper57_pwm_slice, pwm7,
    mmd_bl1_ctl_pwm_slice, pwm4,
    mmd_bl2_ctl_pwm_slice, pwm5,
    mmd_bl1_sense_pwm_slice, pwm1,
    mmd_bl2_sense_pwm_slice, pwm3,
    mmd_br0_pwm_slice, pwm0,
    hpd_motor_pwm_slice, pwm0,

    demo_pwm_slice, pwm0
}

// define pwm channel
define_pins! {
    mmd_bl1_sense_channel, channel_a,
    mmd_bl2_sense_channel, channel_a,
    mmd_bl1_ctl_channel, channel_b,
    mmd_bl2_ctl_channel, channel_a,
    mmd_br_channel_a, channel_a,
    mmd_br_channel_b, channel_b,

    hpd_motor_pwm_a_channel, channel_a,
    hpd_motor_pwm_b_channel, channel_b,

    mmd_motor42_step0_channel, channel_b,
    mmd_motor42_step1_channel, channel_b,
    mmd_stepper57_step_channel, channel_a,

    demo_pwm_channel_a, channel_a,
    demo_pwm_channel_b, channel_b
}
