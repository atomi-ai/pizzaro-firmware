use generic::atomi_error::AtomiError;
use generic::atomi_proto::HpdCommand;

pub fn process_hpd_message(msg: HpdCommand) -> Result<HpdCommand, AtomiError> {
    match msg {
        HpdCommand::HpdPing => Ok(HpdCommand::HpdPong),
        _ => Err(AtomiError::UnaccepableCommand),
    }
}
