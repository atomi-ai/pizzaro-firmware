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

    MmdHome,
    MmdMoveTo { position : i32 },
    MmdMoveToRelative { steps: i32 },

    MmdBusy,
    MmdOnMove,
    MmdInHoming,
}

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, Format)]
pub enum HpdCommand {
    HpdError,
    HpdPing,
    HpdPong,

    HpdHome,
    HpdMoveTo { position: i32 },
    HpdMoveToRelative { distance: i32 },

    // Return status
    HpdBusy,
}

pub fn wrap_result_into_proto(res: Result<AtomiProto, AtomiError>) -> AtomiProto {
    res.unwrap_or_else(|err| AtomiProto::AtomiError(err))
}