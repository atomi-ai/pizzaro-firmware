use serde::{Deserialize, Serialize};

#[derive(
    Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, defmt::Format, Serialize, Deserialize,
)]
pub enum MmdStatus {
    Unavailable, // need to home first.
    Available,

    InHoming,
    InMoving,
}
