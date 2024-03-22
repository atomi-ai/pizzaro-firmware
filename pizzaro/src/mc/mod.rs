use crate::bsp::{McUartDirPinType, McUartType, McUiUartType};

pub mod system_executor;
pub mod touch_screen;

pub type UartType = McUartType;
pub type UartDirType = McUartDirPinType;
pub type UiUartType = McUiUartType;
