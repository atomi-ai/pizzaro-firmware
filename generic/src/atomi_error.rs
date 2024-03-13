use serde::{Deserialize, Serialize};
use crate::mmd_status::MmdStatus;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, defmt::Format, Serialize, Deserialize)]
pub enum AtomiError {
    IgnoredMsg,
    UnaccepableCommand,

    UartReadError,
    UartReadTimeout,
    UartWriteError,
    UartInvalidData,
    UartInvalidInput,

    UsbCtrlInvalidInput,
    UsbCtrlWriteError,
    UnsupportMcCommand,
    DataConvertError,

    MmdUnavailable(MmdStatus),
    MmdStepperHomingError,
    MmdStepperNeedToHome,
    MmdLinearStepperUnrelated,
    MmdMoveWithZeroSpeed,
    MmdPinError,
    MmdNotAcceptedPosition,

    HpdUnavailable,
    HpdNotHomed,
    HpdNeedToHome,
    HpdCannotStart,

    // Used by AtomiProto parsing
    NotIntStr,

}
