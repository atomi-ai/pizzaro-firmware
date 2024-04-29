use crate::bsp::board_mc_release_sb::{McUartDirPinType, McUartType, McUiUartType};

pub mod system_executor;
pub mod touch_screen;

pub type UartType = McUartType;
pub type UartDirType = McUartDirPinType;
pub type UiUartType = McUiUartType;
