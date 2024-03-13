// (by Zephyr): added this file for not touching the complex
// dependencies of PizzaroProtocol.
use defmt::Format;
use serde::{Deserialize, Serialize};
use crate::atomi_error::AtomiError;

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, defmt::Format)]
pub enum AtomiProto {
    Unknown,
    Status,

    Mc(McCommand),
    Mmd(MmdCommand),
    Hpd(HpdCommand),

    AtomiError(AtomiError),
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum McCommand {
    McError,
    McPing,
    McPong,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum MmdCommand {
    MmdError,
    MmdPing,
    MmdPong,
    MmdAck,

    MmdLinearStepper(LinearStepperCommand),

    // TODO(zephyr): Remove enums below.
    MmdInHoming,
    MmdOnMove,
    MmdBusy,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum LinearStepperCommand {
    Home,
    MoveTo { position : i32 },
    MoveToRelative { steps: i32 },
    DummyWait { seconds: i32 },
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum LinearStepperResponse {
    Error(AtomiError),
    Done,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum HpdCommand {
    HpdError,
    HpdPing,
    HpdPong,
    HpdAck,

    HpdLinearBull(LinearBullCommand),

    // Return status
    HpdBusy,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum LinearBullCommand {
    Home,
    MoveTo { position: i32 },
    MoveToRelative { distance: i32 },
    DummyWait { seconds: i32 },
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum LinearBullResponse {
    Error(AtomiError),
    Done,
}

pub fn wrap_result_into_proto(res: Result<AtomiProto, AtomiError>) -> AtomiProto {
    res.unwrap_or_else(|err| AtomiProto::AtomiError(err))
}