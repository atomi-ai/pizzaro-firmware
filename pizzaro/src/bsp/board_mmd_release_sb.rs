/// BSP version: Spring_Begins
///
/// TODO(zephyr): Add schematic pdf below.
///
/// MMD Schematic: [[url]]
use crate::common::brush_motor_patch::BrushMotorPatched;
use crate::common::brushless_motor::BrushlessMotor;
use crate::common::pwm_stepper::{PwmChannels, PwmStepper};
use crate::define_pins;
use rp2040_hal::gpio::bank0::{Gpio11, Gpio18, Gpio21, Gpio22, Gpio26, Gpio27, Gpio4, Gpio5};
use rp2040_hal::gpio::{
    DynPinId, FunctionSio, FunctionSioOutput, FunctionUart, Pin, PullDown, PullUp, SioInput,
    SioOutput,
};
use rp2040_hal::pac::{Interrupt, UART1};
use rp2040_hal::pwm::{Pwm0, Pwm1, Pwm4, Pwm5, Pwm6};
use rp2040_hal::uart::{Enabled, UartPeripheral};

pub type MmdUartPins = (Pin<Gpio4, FunctionUart, PullDown>, Pin<Gpio5, FunctionUart, PullDown>);
pub type MmdUartType = UartPeripheral<Enabled, UART1, MmdUartPins>;
pub type MmdUartDirPinType = Pin<Gpio21, FunctionSioOutput, PullUp>;

/// 使用第二个通道连接驱动传送带旋转的电机
// 42 motor1(and check pins defined in macros)
pub type MmdStepper42_1EnablePinType = Pin<DynPinId, FunctionSio<SioOutput>, PullDown>;
pub type ConveyorBeltRotationMotorType = PwmStepper<Pwm1, MmdStepper42_1EnablePinType>;
#[allow(non_upper_case_globals)]
pub const MmdMotor42Step1Channel: PwmChannels = PwmChannels::channel_b;

// bl0
#[allow(non_upper_case_globals)]
pub const MmdBrushlessMotor0Channel: PwmChannels = PwmChannels::channel_b;
// bl1
#[allow(non_upper_case_globals)]
pub const MmdBrushlessMotor1Channel: PwmChannels = PwmChannels::channel_a;
// brush motor
#[allow(non_upper_case_globals)]
pub const MmdBrushMotorChannel: PwmChannels = PwmChannels::channel_a;

// 42 motor0(and check pins defined in macros)
/// 使用第一个通道连接驱动伸缩的电机
pub type MmdStepper42_0EnablePinType = Pin<Gpio22, FunctionSio<SioOutput>, PullDown>;
pub type MmdStepper42_0DirPinType = Pin<Gpio18, FunctionSio<SioOutput>, PullDown>;
pub type MmdStepper42_0StepPinType = Pin<Gpio11, FunctionSio<SioOutput>, PullDown>;
pub type MmdLimitSwitch0 = Pin<Gpio26, FunctionSio<SioInput>, PullDown>;
pub type MmdLimitSwitch1 = Pin<Gpio27, FunctionSio<SioInput>, PullDown>;

// 57 motor(and check pins defined in macros)
#[allow(non_upper_case_globals)]
pub const MmdMotor57StepChannel: PwmChannels = PwmChannels::channel_b;
pub type MmdStepper57EnablePinType = Pin<DynPinId, FunctionSio<SioOutput>, PullDown>;
pub type MmdPresserMotorType = PwmStepper<Pwm6, MmdStepper57EnablePinType>;

pub type MmdDisperser0MotorType = BrushlessMotor<Pwm4>;
pub type MmdDisperser1MotorType = BrushlessMotor<Pwm5>;

pub type MmdPresserMotorEnablePinType = Pin<DynPinId, FunctionSio<SioOutput>, PullDown>;
pub type MmdPeristalicPumpMotorType = BrushMotorPatched<Pwm0, MmdPresserMotorEnablePinType>;

define_pins! {
    mmd_uart, UART1
}

pub fn mmd_uart_irq() -> Interrupt {
    Interrupt::UART1_IRQ
}

define_pins! {
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
    mmd_stepper42_step0, gpio11, // PWM5.channel_b
    mmd_stepper42_dir0, gpio18,
    mmd_stepper42_nEN0, gpio22,
    mmd_stepper42_diag0, gpio28,
    // 42 motor1
    mmd_stepper42_step1, gpio19, // PWM1.channel_b
    mmd_stepper42_dir1, gpio23,
//    mmd_stepper42_nEN1, gpio25,	// not exists, in v1.5.1, 42_nEN0/1 are connected to gpio22
    mmd_stepper42_diag1, gpio29,
    // 57 motor
    mmd_stepper57_nEN, gpio24,
    mmd_stepper57_step, gpio13,	// PWM6.channel_b
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
    mmd_limit1, gpio27
}

#[macro_export]
macro_rules! mmd_stepper42_nEN1 {
    ($pins:expr) => {
        None
    };
}

// define pwm slices
define_pins! {
    mmd_motor42_pwm_slice0, pwm5,
    mmd_motor42_pwm_slice1, pwm1,
    mmd_stepper57_pwm_slice, pwm6,
    mmd_bl1_ctl_pwm_slice, pwm4,
    mmd_bl2_ctl_pwm_slice, pwm5,
    mmd_bl1_sense_pwm_slice, pwm1,
    mmd_bl2_sense_pwm_slice, pwm3,
    mmd_br0_pwm_slice, pwm0
}

// define pwm channel
define_pins! {
    mmd_bl1_sense_channel, channel_a,
    mmd_bl2_sense_channel, channel_a,
    mmd_bl1_ctl_channel, channel_b,
    mmd_bl2_ctl_channel, channel_a,
    mmd_br_channel_a, channel_a,
    mmd_br_channel_b, channel_b,

    mmd_motor42_step0_channel, channel_b,
    mmd_motor42_step1_channel, channel_b,
    mmd_stepper57_step_channel, channel_b
}
