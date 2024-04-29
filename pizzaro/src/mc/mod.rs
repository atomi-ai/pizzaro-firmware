use crate::bsp::board_mc_release_sb::{McUartDirPinType, McUartType, McUiUartType};
use crate::common::consts::UI_UART_MAX_RESPONSE_LENGTH;
use crate::common::global_timer::Delay;
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
use crate::common::uart::uart_read;
use crate::mc::system_executor::{system_executor_input_mq, system_executor_output_mq};
use crate::mc::touch_screen::TouchScreenEnum;
use defmt::{debug, error, info, warn, Debug2Format};
use fugit::ExtU64;
use generic::atomi_error::AtomiError;
use generic::atomi_proto::{
    wrap_result_into_proto, AtomiProto, McCommand, McSystemExecutorResponse,
};
use rp_pico::hal;
use usbd_serial::SerialPort;

pub mod system_executor;
pub mod touch_screen;

pub type UartType = McUartType;
pub type UartDirType = McUartDirPinType;
pub type UiUartType = McUiUartType;

pub static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

static mut FROM_PC_MESSAGE_QUEUE: Once<MessageQueueWrapper<AtomiProto>> = Once::new();
pub fn get_mc_mq() -> &'static mut MessageQueueWrapper<AtomiProto> {
    unsafe { FROM_PC_MESSAGE_QUEUE.get_mut() }
}

pub async fn process_ui_screen(mut uart: McUiUartType) {
    let mut buffer = [0u8; UI_UART_MAX_RESPONSE_LENGTH];
    let mut buffer_len = 0;
    loop {
        Delay::new(1.millis()).await;
        if uart_read(&mut uart, &mut buffer[buffer_len..buffer_len + 1]).await.is_err() {
            continue;
        }
        buffer_len += 1;

        match TouchScreenEnum::parse(&buffer[..buffer_len]) {
            Ok(TouchScreenEnum::Button { screen_id, object_id, clicked }) => {
                buffer_len = 0;
                // info!(
                //     "btn clicked, screen_id: {}, obj_id:{}, click:{}",
                //     screen_id, object_id, clicked
                // );
                if screen_id == 0 && object_id == 1 && clicked {
                    // engage btn pressed down
                    get_mc_mq().enqueue(AtomiProto::Mc(McCommand::SystemRun(
                        generic::atomi_proto::McSystemExecutorCmd::ExecuteOneFullRun,
                    )))
                } else if screen_id == 0 && object_id == 2 && clicked {
                    // stop btn pressed, FIXME: currently run init instead
                    get_mc_mq().enqueue(AtomiProto::Mc(McCommand::SystemRun(
                        generic::atomi_proto::McSystemExecutorCmd::InitSystem,
                    )))
                }
            }
            Ok(_) => {
                buffer_len = 0;
            }
            _ => {
                if buffer_len >= UI_UART_MAX_RESPONSE_LENGTH {
                    // 缓冲区溢出,清空缓冲区
                    buffer_len = 0;
                }
            }
        }
    }
}

pub async fn process_messages() {
    // TODO(zephyr): Do we need to keep connection alive?
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
    // TODO(zephyr): Rethink: do we need to use AtomicBool for the lock?
    let mut system_locked = false;

    loop {
        Delay::new(1.millis()).await;

        if let Some(resp) = system_executor_output_mq().dequeue() {
            info!("[MC] get response from system executor: {}", resp);
            system_locked = false;
        }

        let t = get_mc_mq().dequeue();
        // TODO(zephyr): simplify the code below.
        if t.is_some() {
            info!("[MC] get msg from queue: {}", t);
        }
        let msg = match t {
            None => continue, // no data, ignore
            Some(AtomiProto::Mc(McCommand::McPing)) => Ok(AtomiProto::Mc(McCommand::McPong)),
            Some(AtomiProto::Mc(McCommand::SystemRun(cmd))) => {
                if system_locked {
                    Err(AtomiError::McLockedForSystemRun)
                } else {
                    system_locked = true;
                    system_executor_input_mq().enqueue(AtomiProto::Mc(McCommand::SystemRun(cmd)));
                    Ok(AtomiProto::Mc(McCommand::McAck))
                }
            }
            Some(msg) => {
                if system_locked {
                    Err(AtomiError::McLockedForSystemRun)
                } else {
                    debug!("[MC] add msg to system_executor_input_mq: {:?}", Debug2Format(&msg));
                    system_executor_input_mq().enqueue(msg);
                    wait_for_output_mq().await
                }
            }
        };
        info!("Processed result: {}", msg);

        let result = (|| -> Result<(), AtomiError> {
            let wrapped_msg = wrap_result_into_proto(msg);
            let binding =
                postcard::to_allocvec(&wrapped_msg).map_err(|_| AtomiError::DataConvertError)?;
            debug!("Data to send to PC: {}, wrapped_msg: {}", Debug2Format(&binding), wrapped_msg);
            let mut wr_ptr = binding.as_slice();
            while !wr_ptr.is_empty() {
                match serial.write(wr_ptr) {
                    Ok(len) => {
                        debug!("process_mc_message() 5.4, len = {}", len);
                        if len > wr_ptr.len() {
                            error!("process_messages() 5.5: overflow happens");
                            return Err(AtomiError::UsbCtrlWriteError);
                        }
                        wr_ptr = &wr_ptr[len..]
                    }
                    Err(_) => Err(AtomiError::UsbCtrlWriteError)?,
                };
            }
            Ok(())
        })();

        match result {
            Ok(_) => {
                debug!("Successfully send data back through USBCTRL");
            }
            Err(err) => {
                error!("Errors in sending data back through USBCTRL, {}", err)
            }
        }
    }
}

pub fn expect_result(_actual: AtomiProto, _expected: AtomiProto) -> Result<(), AtomiError> {
    // TODO(zephyr): Figure a way to expect the correct result.
    Ok(())
}

pub fn error_or_done(
    res: Result<(), AtomiError>,
    mq_out: &mut MessageQueueWrapper<McSystemExecutorResponse>,
) {
    info!("Got result: {}", res);
    match res {
        Ok(_) => mq_out.enqueue(McSystemExecutorResponse::Done),
        Err(err) => mq_out.enqueue(McSystemExecutorResponse::Error(err)),
    }
}

async fn wait_for_output_mq() -> Result<AtomiProto, AtomiError> {
    let loop_times = 1_000;
    for _ in 0..loop_times {
        if let Some(McSystemExecutorResponse::ForwardResponse(resp)) =
            system_executor_output_mq().dequeue()
        {
            info!("wait_for_forward_dequeue() 5: got response: {}", resp);
            return Ok(resp);
        }
        Delay::new(1.millis()).await;
    }
    warn!("Not getting correct forward response, loop_times = {}", loop_times);
    // timeout
    Err(AtomiError::McForwardTimeout)
}
