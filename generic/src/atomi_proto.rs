// (by Zephyr): added this file for not touching the complex
// dependencies of PizzaroProtocol.
use crate::atomi_error::AtomiError;
use defmt::Format;
use serde::{Deserialize, Serialize};

// #[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, defmt::Format)]
// pub enum AtomiAutorun {
//     Start,
//     Stop,
//     Wait { seconds: i32 },
//     Done,
// }

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum AtomiProto {
    Unknown,
    Status,
    // Autorun(AtomiAutorun),
    Mc(McCommand),
    Mmd(MmdCommand),
    Hpd(HpdCommand),
    Dtu(DtuCommand),

    AtomiError(AtomiError),
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub struct AtomiErrorWithCanId {
    error_can_id: u16,
    error: AtomiError,
}

impl AtomiErrorWithCanId {
    pub fn new(error_can_id: u16, error: AtomiError) -> Self {
        AtomiErrorWithCanId { error_can_id, error }
    }
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum McCommand {
    McError,
    McPing,
    McPong,

    SystemRun(McSystemExecutorCmd),
    FullRun,
    McAck,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum McSystemExecutorCmd {
    WeightSensorInit,
    GetWeight,

    ExecuteOneFullRun,
    InitSystem,
    MakePizza,

    StopSystem,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum McSystemExecutorResponse {
    ForwardResponse(AtomiProto),
    FinishedOneFullRun,
    Error(AtomiError),
    Weight(i32),
    Done,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum MmdCommand {
    MmdError(AtomiError),
    MmdPing,
    MmdPong,
    MmdAck,

    MmdStop,
    MmdLinearStepper(StepperCommand),
    MmdRotationStepper(RotationStepperCommand),
    MmdDisperser(DispenserCommand),
    MmdPeristalticPump(PeristalticPumpCommand),

    // TODO(zephyr): Remove enums below.
    MmdInHoming,
    MmdOnMove,
    MmdBusy,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum DtuCommand {
    DtuPing,
    DtuPong,
    DtuAck,
    DtuStop,

    DtuLinear(StepperCommand),

    DtuBusy,
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
pub enum StepperCommand {
    Home,                                           // wait
    MoveTo { position: i32, speed: u32 },           // wait
    MoveToRelative { steps: i32, speed: u32 },      // wait
    MoveToRelativeForce { steps: i32, speed: u32 }, // wait
    WaitIdle,
    GetTriggerStatus,
    Off,
    DummyWait { seconds: i32 },
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum RotationStepperResponse {
    Error(AtomiError),
    Done,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum StepperResponse {
    TriggerStatus(TriggerStatusResponse),
    Error(AtomiError),
    Done,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum HpdCommand {
    HpdError(AtomiError),
    HpdPing,
    HpdPong,
    HpdAck,

    HpdStop,
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

// This should be an idempotent function: f(f(x)) = f(x)
pub fn wrap_result_into_proto(res: Result<AtomiProto, AtomiError>) -> AtomiProto {
    res.unwrap_or_else(AtomiProto::AtomiError)
}
