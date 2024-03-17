//pub use crate::bsp::board_early_release::*;
#[macro_export]
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

// #[cfg(feature = "bsp_early_release")]
// pub use super::board_early_release::*;

// #[cfg(feature = "bsp_v1_5_4")]
// pub use super::board_v1_5_4::*;
