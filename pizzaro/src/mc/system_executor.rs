use core::future::Future;
use defmt::{error, info};
use fugit::ExtU64;
use generic::atomi_error::AtomiError;
use generic::atomi_proto::{AtomiProto, DispenserCommand, HpdCommand, LinearBullCommand, LinearStepperCommand, LinearStepperResponse, McSystemExecutorCmd, McSystemExecutorResponse, MmdCommand, RotationStepperCommand};
use crate::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use crate::common::global_timer::Delay;
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
use crate::common::uart_comm;
use crate::common::uart_comm::UartComm;
use crate::mc::{UartDirType, UartType};

static mut SYSTEM_EXECUTOR_INPUT_MQ_ONCE: Once<MessageQueueWrapper<McSystemExecutorCmd>> =
    Once::new();
static mut SYSTEM_EXECUTOR_OUTPUT_MQ_ONCE: Once<MessageQueueWrapper<McSystemExecutorResponse>> =
    Once::new();
pub fn system_executor_input_mq() -> &'static mut MessageQueueWrapper<McSystemExecutorCmd> {
    unsafe { SYSTEM_EXECUTOR_INPUT_MQ_ONCE.get_mut() }
}
pub fn system_executor_output_mq() -> &'static mut MessageQueueWrapper<McSystemExecutorResponse> {
    unsafe { SYSTEM_EXECUTOR_OUTPUT_MQ_ONCE.get_mut() }
}

pub async fn forward(
    uart_comm: &mut UartComm<'_, UartDirType, UartType>,
    msg: AtomiProto,
) -> Result<AtomiProto, AtomiError> {
    info!("Forward msg: {}", msg);
    if let Err(e) = uart_comm.send(msg) {
        error!("Error in sending command, err: {:?}", e);
        return Err(AtomiError::UartWriteError);
    }

    // 异步读取响应长度 or timeout
    uart_comm.recv_timeout::<AtomiProto>(500.millis()).await
}

async fn wait_for_linear_stepper_available(uart_comm: &mut UartComm<'_, UartDirType, UartType>) -> Result<(), AtomiError> {
    loop {
        match forward(uart_comm, AtomiProto::Mmd(
            MmdCommand::MmdLinearStepper(LinearStepperCommand::WaitIdle))).await {
            Ok(AtomiProto::Mmd(MmdCommand::MmdAck)) => {
                // continue for the next step;
                break;
            }
            Err(err) => return Err(AtomiError::MmdLinearStepperWaitIdleError),
            _ => {}  // continue to wait
        };
    }
    Ok(())
}

// TODO(lv): create an executor, put uart_comm into it. Then we don't need to pass uart_comm param.
pub async fn execute_all_commands(uart_comm: &mut UartComm<'_, UartDirType, UartType>) -> Result<(), AtomiError> {
    // TODO(lv): expect the response?
    let _ = forward(uart_comm, AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::MoveTo {
        position: 0,
    }))).await?;
    wait_for_linear_stepper_available(uart_comm).await?;

    let _ = forward(uart_comm, AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::MoveTo {
        position: 0,
    }))).await?;

    let _ = forward(uart_comm, AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
        RotationStepperCommand::SetPresserRotation { speed: 200 },
    ))).await?;

    let _ = forward(uart_comm, AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::MoveTo {
        position: 200,
    }))).await?;
    wait_for_linear_stepper_available(uart_comm).await?;

    let _ = forward(uart_comm, AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
        RotationStepperCommand::SetConveyorBeltRotation { speed: 200 },
    ))).await?;

    let _ = forward(uart_comm, AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
        idx: 0,
        speed: 1000,
    }))).await?;

    let _ = forward(uart_comm, AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::MoveTo {
        position: 0,
    }))).await?;
    wait_for_linear_stepper_available(uart_comm).await?;

    let _ = forward(uart_comm, AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
        RotationStepperCommand::SetConveyorBeltRotation { speed: 0 },
    ))).await?;

    let _ = forward(uart_comm, AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
        idx: 0,
        speed: 0,
    }))).await?;

    Ok(())
}

pub async fn process_mc_one_full_run(uart_comm: &mut UartComm<'_, UartDirType, UartType>) {
    info!("process_mc_one_full_run() 0");
    let mq_in = system_executor_input_mq();
    let mq_out = system_executor_output_mq();
    loop {
        Delay::new(1.millis()).await;
        let res = match mq_in.dequeue() {
            Some(McSystemExecutorCmd::ExecuteOneFullRun) => {
                let x = execute_all_commands(uart_comm).await;
            }
            _ => continue,
        };
        mq_out.enqueue(McSystemExecutorResponse::FinishedOneFullRun)
    }
}
