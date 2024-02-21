// (by Zephyr): added this file for not touching the complex
// dependencies of PizzaroProtocol.
use defmt::Format;
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, PartialEq, Serialize, Deserialize, Debug, defmt::Format)]
pub enum AtomiProto {
    Unknown,
    Status,

    Mc(McCommand),
    Mmd(MmdCommand),
    Hpd(HpdCommand),
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
}
