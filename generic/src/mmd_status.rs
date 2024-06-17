use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Serialize, Deserialize)]
pub enum MmdStatus {
    Unavailable, // need to home first.
    Available,

    InHoming,
    InMoving,
}
