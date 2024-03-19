// (by Zephyr): added this file for not touching the complex
// dependencies of PizzaroProtocol.
use crate::atomi_error::AtomiError;
use defmt::Format;
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, defmt::Format)]
pub enum AtomiAutorun {
    Start,
    Stop,
    Wait { seconds: i32 },
    Done,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, defmt::Format)]
pub enum AtomiProto {
    Unknown,
    Status,
    Autorun(AtomiAutorun),

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
    MmdRotationStepper(RotationStepperCommand),
    MmdDisperser(DispenserCommand),
    MmdPeristalticPump(PeristalticPumpCommand),

    // TODO(zephyr): Remove enums below.
    MmdInHoming,
    MmdOnMove,
    MmdBusy,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub struct TriggerStatusResponse {
    pub left: bool,
    pub right: bool,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum PeristalticPumpCommand {
    SetRotation { speed: i32 }, // 0-1000 => 0.0% - 100.0%
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum PeristalticPumpResponse {
    Error(AtomiError),
    Done,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum DispenserCommand {
    SetRotation { idx: usize, speed: i32 }, // 0-1000 => 0.0% - 100.0%
    ResetCounter { idx: usize },
    GetCounter { idx: usize },
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum DispenserResponse {
    Error { idx: usize, err: AtomiError },
    Done { idx: usize },
    Counter { idx: usize, value: i32 },
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum RotationStepperCommand {
    SetConveyorBeltRotation { speed: i32 }, // -1000 ~ +1000 => 100.0%(left) ~ 0 ~ 100.0%(right)
    SetPresserRotation { speed: i32 },      // -1000 ~ +1000 => 100.0%(left) ~ 0 ~ 100.0%(right)
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum LinearStepperCommand {
    Home,                               // wait
    MoveTo { position: i32 },           // wait
    MoveToRelative { steps: i32 },      // wait
    MoveToRelativeForce { steps: i32 }, // wait
    WaitIdle,
    GetTriggerStatus,
    DummyWait { seconds: i32 },
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum RotationStepperResponse {
    Error(AtomiError),
    Done,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum LinearStepperResponse {
    TriggerStatus(TriggerStatusResponse),
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
    Home,                             // wait
    MoveTo { position: i32 },         // wait
    MoveToRelative { distance: i32 }, // wait
    WaitIdle,
    DummyWait { seconds: i32 },
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum LinearBullResponse {
    Error(AtomiError),
    Done,
}

pub fn wrap_result_into_proto(res: Result<AtomiProto, AtomiError>) -> AtomiProto {
    res.unwrap_or_else(AtomiProto::AtomiError)
}
