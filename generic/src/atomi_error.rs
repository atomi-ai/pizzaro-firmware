use crate::mmd_status::MmdStatus;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, defmt::Format)]
pub enum AtomiError {
    IgnoredMsg,
    UnaccepableCommand,

    UartReadError,
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
    MmdMoveWithZeroSpeed,
    MmdPinError,
    MmdNotAcceptedPosition,

    HpdNotHomed,
    HpdNeedToHome,
    HpdCannotStart,

    // Used by AtomiProto parsing
    NotIntStr,

}
