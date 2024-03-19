use crate::atomi_error::AtomiError;
use crate::atomi_proto::{
    AtomiAutorun, AtomiProto, DispenserCommand, HpdCommand, LinearBullCommand,
    LinearStepperCommand, McCommand, MmdCommand, PeristalticPumpCommand, RotationStepperCommand,
};

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

        Some("autorun") => AtomiProto::Autorun(AtomiAutorun::Start),
        Some("autostop") => AtomiProto::Autorun(AtomiAutorun::Stop),

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
        Some("trigger") => AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
            LinearStepperCommand::GetTriggerStatus,
        )),
        Some("home") => AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::Home)),
        Some("force_move") => {
            if let Ok(steps) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                    LinearStepperCommand::MoveToRelativeForce { steps },
                ))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("move_rel") => {
            if let Ok(steps) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                    LinearStepperCommand::MoveToRelative { steps },
                ))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("move_to") => {
            if let Ok(position) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::MoveTo {
                    position,
                }))
            } else {
                AtomiProto::Unknown
            }
        }

        // belt rotating
        Some("belt_on") => AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetConveyorBeltRotation { speed: 290 },
        )),
        Some("belt_off") => AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetConveyorBeltRotation { speed: 0 },
        )),
        Some("belt_spd") => {
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
            RotationStepperCommand::SetPresserRotation { speed: 1000 },
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
        Some("pp_on") => AtomiProto::Mmd(MmdCommand::MmdPeristalticPump(
            PeristalticPumpCommand::SetRotation { speed: 1000 },
        )),
        Some("pp_off") => AtomiProto::Mmd(MmdCommand::MmdPeristalticPump(
            PeristalticPumpCommand::SetRotation { speed: 0 },
        )),
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
            AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::WaitIdle))
        }

        Some("dummy") => {
            if let Ok(seconds) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                    LinearStepperCommand::DummyWait { seconds },
                ))
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

        Some("move_rel") => {
            if let Ok(distance) = parse_int(tokens.next()) {
                AtomiProto::Hpd(HpdCommand::HpdLinearBull(
                    LinearBullCommand::MoveToRelative { distance },
                ))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("move_to") => {
            if let Ok(position) = parse_int(tokens.next()) {
                AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::MoveTo {
                    position,
                }))
            } else {
                AtomiProto::Unknown
            }
        }

        Some("wait_idle") => {
            AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::WaitIdle))
        }

        Some("dummy") => {
            if let Ok(seconds) = parse_int(tokens.next()) {
                AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::DummyWait {
                    seconds,
                }))
            } else {
                AtomiProto::Unknown
            }
        }
        _ => AtomiProto::Unknown,
    }
}
