use crate::atomi_error::AtomiError;
use crate::atomi_proto::{AtomiProto, HpdCommand, LinearStepperCommand, McCommand, MmdCommand};

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
        _ => AtomiProto::Mc(McCommand::McError),
    }
}

fn parse_int(token: Option<&str>) -> Result<i32, AtomiError> {
    if let Some(str) = token {
        if let Ok(v) = str.parse::<i32>() {
            return Ok(v)
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
        Some("home") => AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::Home)),
        Some("move_rel") => {
            if let Ok(steps) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                    LinearStepperCommand::MoveToRelative {steps}))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("move_to") => {
            if let Ok(position) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                    LinearStepperCommand::MoveTo {position}))
            } else {
                AtomiProto::Unknown
            }
        }
        Some("dummy") => {
            if let Ok(seconds) = parse_int(tokens.next()) {
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                    LinearStepperCommand::DummyWait {seconds}))
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
        Some("home") => AtomiProto::Hpd(HpdCommand::HpdHome),
        Some("move_rel") => {
            if let Ok(distance) = parse_int(tokens.next()) {
                AtomiProto::Hpd(HpdCommand::HpdMoveToRelative {distance})
            } else {
                AtomiProto::Unknown
            }
        }
        Some("move_to") => {
            if let Ok(position) = parse_int(tokens.next()) {
                AtomiProto::Hpd(HpdCommand::HpdMoveTo {position})
            } else {
                AtomiProto::Unknown
            }
        }
        _ => AtomiProto::Unknown,
    }
}
