use crate::mmd_status::MmdStatus;
use serde::{Deserialize, Serialize};

#[derive(
    Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, defmt::Format, Serialize, Deserialize,
)]
pub enum AtomiError {
    IgnoredMsg,
    UnaccepableCommand,

    CanCobIdError,
    CanFrameError,
    CanTransmitError,

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
    MmdStopped,   // 这个是异常退出的路径
    MmdStopError, // 这个是用于MC强制MMD stop时候产生的错误

    HpdUnavailable,
    HpdNotHomed,
    HpdNeedToHome,
    HpdCannotStart,
    HpdStopped,

    // Used by AtomiProto parsing
    NotIntStr,

    // Used by state
    StateOverflow,
}
