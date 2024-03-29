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
