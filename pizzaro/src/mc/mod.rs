use crate::bsp::{McUartDirPinType, McUartType, McUiUartType};
use crate::common::uart_comm::UartComm;

pub mod touch_screen;
pub mod system_executor;

pub type UartType = McUartType;
pub type UartDirType = McUartDirPinType;
pub type UiUartType = McUiUartType;
