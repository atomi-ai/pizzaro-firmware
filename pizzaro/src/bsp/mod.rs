mod board_helper;

#[cfg(feature = "bsp_early_release")]
pub mod board_early_release;
#[cfg(feature = "bsp_early_release")]
pub use board_early_release::*;

// #[cfg(feature = "bsp_v1_5_4")]
// mod board_v1_5_4;
// #[cfg(feature = "bsp_v1_5_4")]
// pub use board_v1_5_4::*;

// pub use self::board::*;
pub mod config;
