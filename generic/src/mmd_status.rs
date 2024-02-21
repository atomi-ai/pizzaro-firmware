#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, defmt::Format)]
pub enum MmdStatus {
    Unavailable,  // need to home first.
    Available,

    InHoming,
    InMoving,
}
