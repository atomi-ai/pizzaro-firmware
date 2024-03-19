use crate::mmd_status::MmdStatus;
use serde::{Deserialize, Serialize};

#[derive(
    Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, defmt::Format, Serialize, Deserialize,
)]
pub enum AtomiError {
    IgnoredMsg,
    UnaccepableCommand,

    UartReadError,
    UartReadTimeout,
    UartWriteError,
    UartFlushError,
    UartInvalidData,
    UartInvalidInput,
    UartSetDirError,

    UsbCtrlInvalidInput,
    UsbCtrlWriteError,
    UnsupportMcCommand,
    DataConvertError,

    GpioPinError,

    McLockedForSystemRun,
    McForwardTimeout,

    MmdUnavailable(MmdStatus),
    MmdStepperHomingError,
    MmdStepperNeedToHome,
    MmdLinearStepperUnrelated,
    MmdMoveWithZeroSpeed,
    MmdNotAcceptedPosition,
    MmdUnknownDispenserIdx,
    MmdCannotStart,
    MmdLinearStepperWaitIdleError,

    HpdUnavailable,
    HpdNotHomed,
    HpdNeedToHome,
    HpdCannotStart,

    // Used by AtomiProto parsing
    NotIntStr,

    // Used by state
    StateOverflow,
}
