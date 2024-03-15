// use rp2040_hal::gpio::bank0::{Gpio11, Gpio18, Gpio22, Gpio26, Gpio27, Gpio4, Gpio5};
// use rp2040_hal::gpio::{
//     DynPinId, FunctionSio, FunctionUart, Pin, PullDown, PullUp, SioInput, SioOutput,
// };
// use rp2040_hal::pac::UART1;
// use rp2040_hal::uart::{Enabled, UartPeripheral};

// use crate::common::global_timer::DelayCreator;
// use crate::mmd::linear_stepper::LinearStepper;

// pub type EnablePinType = Pin<Gpio22, FunctionSio<SioOutput>, PullDown>;
// pub type DirPinType = Pin<Gpio18, FunctionSio<SioOutput>, PullDown>;
// pub type StepPinType = Pin<Gpio11, FunctionSio<SioOutput>, PullDown>;
// pub type LimitLeftPinType = Pin<Gpio26, FunctionSio<SioInput>, PullDown>;
// pub type LimitRightPinType = Pin<Gpio27, FunctionSio<SioInput>, PullDown>;
// pub type LinearStepperType = LinearStepper<
//     LimitLeftPinType,
//     LimitRightPinType,
//     EnablePinType,
//     DirPinType,
//     StepPinType,
//     DelayCreator,
// >;

// pub type MmdUartType = UartPeripheral<
//     Enabled,
//     UART1,
//     (
//         Pin<Gpio4, FunctionUart, PullDown>,
//         Pin<Gpio5, FunctionUart, PullDown>,
//     ),
// >;

// pub type MmdUartDirType = Pin<DynPinId, FunctionSio<SioOutput>, PullUp>;

// TODO(zephyr): Move the content to mod.rs, and remove the file.
