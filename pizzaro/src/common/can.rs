use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use can2040::{Can2040, CanFrame};
use defmt::{debug, error, info, Debug2Format, Format};
use embedded_can::{Frame, Id, StandardId};
use generic::atomi_error::AtomiError;
use serde::de::DeserializeOwned;
use serde::Serialize;

use crate::common::once::Once;

static mut CAN_BUS_ONCE: Once<Can2040> = Once::new();
pub fn get_can() -> &'static mut Can2040 {
    unsafe { CAN_BUS_ONCE.get_mut_or_fail() }
}

pub fn init_can_bus(
    core: &mut cortex_m::Peripherals,
    baud_rate: u32,
    can_rx_id: u32,
    can_tx_id: u32,
) {
    unsafe {
        CAN_BUS_ONCE.init_once_with_function(|| {
            can2040::initialize_cbus(core, baud_rate, can_rx_id, can_tx_id)
        });
    }
}

pub fn send_can_message<U: Format + Serialize>(cob_id: u16, msg: U) -> Result<(), AtomiError> {
    info!("send_can_message(): to transmit msg: {}", msg);
    let data = postcard::to_allocvec::<U>(&msg).map_err(|_| AtomiError::UartInvalidInput)?;
    let f =
        CanFrame::new(StandardId::new(cob_id).ok_or(AtomiError::CanCobIdError)?, data.as_slice())
            .ok_or(AtomiError::CanFrameError)?;
    debug!("send_can_message(): to transmit frame: {}", f);
    match <Can2040 as embedded_can::blocking::Can>::transmit(get_can(), &f) {
        Ok(_) => {
            info!("Transmitted package: {:?}", f);
            Ok(())
        }
        Err(err) => {
            error!("Transmit error: {}", err);
            Err(AtomiError::CanTransmitError)?
        }
    }
}

pub fn enqueue_can_frame_if_possible<T>(acceptable_can_id: u16, frame: CanFrame, queue: &mut MessageQueueWrapper<T>)
where
    T: DeserializeOwned,
{
    debug!("frame_to_hpd_msg() : convert can msg: {}", frame);
    // 确认是HPD的消息。
    match frame.id() {
        Id::Standard(sid) => {
            if sid.as_raw() != acceptable_can_id {
                // ignore HPD unrelated msgs.
                return;
            }
        }
        _ => {
            // Unsupport ids.
            return;
        }
    }
    match postcard::from_bytes::<T>(&frame.data()[..frame.dlc()]) {
        Ok(msg) => queue.enqueue(msg),
        Err(err) => {
            error!("frame_to_hpd_msg(): errors in parsing HPD message, err: {}", Debug2Format(&err))
        }
    }
}
