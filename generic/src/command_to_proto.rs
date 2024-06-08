use crate::atomi_error::AtomiError;
use crate::atomi_proto::{
    AsdCommand, AtomiProto, DispenserCommand, DtuCommand, HpdCommand, LinearBullCommand, McCommand,
    McSystemExecutorCmd, MmdCommand, PeristalticPumpCommand, RotationStepperCommand,
    StepperCommand, StepperDriverCommand,
};

const FAST_SPEED: u32 = 100; // steps / second

pub fn parse_protocol(line: &str) -> AtomiProto {
    let mut tokens = line.split_whitespace();
    parse_command(&mut tokens)
}

fn parse_command<'a, I>(tokens: &mut I) -> AtomiProto
where
    I: Iterator<Item = &'a str>,
{
    match tokens.next() {
        Some("mc") => parse_mc_command(tokens),
        Some("mmd") => parse_mmd_command(tokens),
        Some("hpd") => parse_hpd_command(tokens),
        Some("dtu") => parse_dtu_command(tokens),
        Some("asd") => parse_asd_command(tokens),
        _ => AtomiProto::Unknown,
    }
}

fn parse_mc_command<'a, I>(tokens: &mut I) -> AtomiProto
where
    I: Iterator<Item = &'a str>,
{
    match tokens.next() {
        Some("ping") => AtomiProto::Mc(McCommand::McPing),
        Some("pong") => AtomiProto::Mc(McCommand::McPong),

        // TODO(lv): Deprecate the "autorun" command below?
        Some("autorun") => {
            AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::ExecuteOneFullRun))
        }
        Some("weightinit") => {
            AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::WeightSensorInit))
        }
        Some("weight") => AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::GetWeight)),
        Some("init") => AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::InitSystem)),
        Some("make") => AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::MakePizza)),
        Some("ketch_up_test") => {
            AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::KetchUpTest))
        }
        Some("stop") => AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::StopSystem)),
        // Some("autostop") => AtomiProto::Autorun(AtomiAutorun::Stop),
        _ => AtomiProto::Mc(McCommand::McError),
    }
}

fn parse_int(token: Option<&str>) -> Result<i32, AtomiError> {
    if let Some(str) = token {
        if let Ok(v) = str.parse::<i32>() {
            return Ok(v);
        }
    }
    Err(AtomiError::NotIntStr)
}

fn parse_mmd_command<'a, I>(tokens: &mut I) -> AtomiProto
where
    I: Iterator<Item = &'a str>,
{
    match tokens.next() {
        Some("ping") => AtomiProto::Mmd(MmdCommand::MmdPing),
        Some("pong") => AtomiProto::Mmd(MmdCommand::MmdPong),
        Some("stop") => AtomiProto::Mmd(MmdCommand::MmdStop),
        Some("trigger") => {
            AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::GetTriggerStatus))
        }
        Some("home") => AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::Home)),
        Some("stepper_off") => AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::Off)),
        Some("force_move") => {
            if let Ok(steps) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::MoveToRelativeForce {
                    steps,
                    speed: FAST_SPEED,
                }))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("move_rel") => {
            if let Ok(steps) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::MoveToRelative {
                    steps,
                    speed: FAST_SPEED,
                }))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("move_to") => {
            if let Ok(position) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::MoveTo {
                    position,
                    speed: FAST_SPEED,
                }))
            } else {
                AtomiProto::Unknown
            }
        }

        // belt rotating
        Some("belt_on") => AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetConveyorBeltRotation { speed: 100 },
        )),
        Some("belt_off") => AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetConveyorBeltRotation { speed: 0 },
        )),
        Some("belt_spd") => {
            // belt speed: -290..290
            if let Ok(speed) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
                    RotationStepperCommand::SetConveyorBeltRotation { speed },
                ))
            } else {
                AtomiProto::Unknown
            }
        }

        // presser platform rotating
        Some("pr_on") => AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetPresserRotation { speed: 300 },
        )),
        Some("pr_off") => AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetPresserRotation { speed: 0 },
        )),
        Some("pr_spd") => {
            if let Ok(speed) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
                    RotationStepperCommand::SetPresserRotation { speed },
                ))
            } else {
                AtomiProto::Unknown
            }
        }

        // dispenser command: on, off, set speed
        Some("dispenser0_on") => {
            AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
                idx: 0,
                speed: -1000,
            }))
        }
        Some("dispenser1_on") => {
            AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
                idx: 1,
                speed: -1000,
            }))
        }
        Some("dispenser0_off") => {
            AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
                idx: 0,
                speed: 0,
            }))
        }
        Some("dispenser1_off") => {
            AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
                idx: 1,
                speed: 0,
            }))
        }
        Some("dispenser0_spd") => {
            if let Ok(speed) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
                    idx: 0,
                    speed,
                }))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("dispenser1_spd") => {
            if let Ok(speed) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
                    idx: 1,
                    speed,
                }))
            } else {
                AtomiProto::Unknown
            }
        }

        // peristaltic pump command: on, off, set speed
        Some("pp_on") => {
            AtomiProto::Mmd(MmdCommand::MmdPeristalticPump(PeristalticPumpCommand::SetRotation {
                speed: 1000,
            }))
        }
        Some("pp_off") => {
            AtomiProto::Mmd(MmdCommand::MmdPeristalticPump(PeristalticPumpCommand::SetRotation {
                speed: 0,
            }))
        }
        Some("pp_spd") => {
            if let Ok(speed) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdPeristalticPump(
                    PeristalticPumpCommand::SetRotation { speed },
                ))
            } else {
                AtomiProto::Unknown
            }
        }

        Some("wait_idle") => {
            AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::WaitIdle))
        }

        Some("dummy") => {
            if let Ok(seconds) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::DummyWait { seconds }))
            } else {
                AtomiProto::Unknown
            }
        }
        _ => AtomiProto::Unknown,
    }
}

fn parse_hpd_command<'a, I>(tokens: &mut I) -> AtomiProto
where
    I: Iterator<Item = &'a str>,
{
    match tokens.next() {
        Some("ping") => AtomiProto::Hpd(HpdCommand::HpdPing),
        Some("pong") => AtomiProto::Hpd(HpdCommand::HpdPong),
        Some("home") => AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::Home)),
        Some("stop") => AtomiProto::Hpd(HpdCommand::HpdStop),
        Some("get_pos") => {
            AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::GetPosition))
        }
        Some("echo") => {
            if let Ok(idx) = parse_int(tokens.next()) {
                AtomiProto::Hpd(HpdCommand::HpdEcho(idx as u8))
            } else {
                AtomiProto::Unknown
            }
        }

        Some("move_rel") => {
            if let Ok(distance) = parse_int(tokens.next()) {
                AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::MoveToRelative {
                    distance,
                }))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("move_to") => {
            if let Ok(position) = parse_int(tokens.next()) {
                AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::MoveTo { position }))
            } else {
                AtomiProto::Unknown
            }
        }

        Some("wait_idle") => {
            AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::WaitIdle))
        }

        Some("dummy") => {
            if let Ok(seconds) = parse_int(tokens.next()) {
                AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::DummyWait { seconds }))
            } else {
                AtomiProto::Unknown
            }
        }
        _ => AtomiProto::Unknown,
    }
}

fn parse_dtu_command<'a, I>(tokens: &mut I) -> AtomiProto
where
    I: Iterator<Item = &'a str>,
{
    match tokens.next() {
        Some("ping") => AtomiProto::Dtu(DtuCommand::DtuPing),
        Some("stop") => AtomiProto::Dtu(DtuCommand::DtuStop),
        Some("trigger") => AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::GetTriggerStatus)),
        Some("home") => AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::Home)),
        Some("stepper_off") => AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::Off)),
        Some("force_move") => {
            if let Ok(steps) = parse_int(tokens.next()) {
                AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::MoveToRelativeForce {
                    steps,
                    speed: FAST_SPEED,
                }))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("move_rel") => {
            if let Ok(steps) = parse_int(tokens.next()) {
                AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::MoveToRelative {
                    steps,
                    speed: FAST_SPEED,
                }))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("move_to") => {
            if let Ok(position) = parse_int(tokens.next()) {
                if let Ok(speed) = parse_int(tokens.next()) {
                    AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::MoveTo {
                        position,
                        speed: speed as u32,
                    }))
                } else {
                    AtomiProto::Unknown
                }
            } else {
                AtomiProto::Unknown
            }
        }

        Some("wait_idle") => AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::WaitIdle)),

        Some("dummy") => {
            if let Ok(seconds) = parse_int(tokens.next()) {
                AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::DummyWait { seconds }))
            } else {
                AtomiProto::Unknown
            }
        }
        _ => AtomiProto::Unknown,
    }
}

fn parse_asd_command<'a, I>(tokens: &mut I) -> AtomiProto
where
    I: Iterator<Item = &'a str>,
{
    match tokens.next() {
        Some("ping") => AtomiProto::Asd(AsdCommand::AsdPing),
        Some("stop") => AtomiProto::Asd(AsdCommand::AsdStop),
        Some("move_rel") => (|| {
            let steps = parse_int(tokens.next())?;
            let speed = parse_int(tokens.next())?;
            Ok::<AtomiProto, AtomiError>(AtomiProto::Asd(AsdCommand::AsdStepper(
                StepperDriverCommand::MoveToRelative { steps, speed: speed as u32 },
            )))
        })()
        .unwrap_or(AtomiProto::Unknown),
        Some("check_status") => {
            AtomiProto::Asd(AsdCommand::AsdStepper(StepperDriverCommand::CheckStatus))
        }
        _ => AtomiProto::Unknown,
    }
}
