use defmt::{error, info};
use fugit::ExtU64;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{AtomiProto, DispenserCommand, HpdCommand, LinearBullCommand, LinearStepperCommand, McSystemExecutorCmd, McSystemExecutorResponse, MmdCommand, RotationStepperCommand, wrap_result_into_proto};

use crate::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use crate::common::global_timer::Delay;
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
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

pub struct McSystemExecutor {
    uart: UartType,
    uart_dir: Option<UartDirType>,
}

impl McSystemExecutor {
    pub fn new(uart: UartType, uart_dir: Option<UartDirType>) -> Self {
        Self {
            uart, uart_dir,
        }
    }

    fn get_uart_comm(&mut self) -> UartComm<'_, UartDirType, UartType> {
        UartComm::new(&mut self.uart, &mut self.uart_dir, UART_EXPECTED_RESPONSE_LENGTH)
    }

    pub async fn forward(&mut self, msg: AtomiProto) -> Result<AtomiProto, AtomiError> {
        info!("Forward msg: {}", msg);
        let mut uart_comm = self.get_uart_comm();
        if let Err(e) = uart_comm.send(msg) {
            error!("Error in sending command, err: {:?}", e);
            return Err(AtomiError::UartWriteError);
        }

        // 异步读取响应长度 or timeout
        uart_comm.recv_timeout::<AtomiProto>(500.millis()).await
    }

    async fn wait_for_linear_stepper_available(&mut self) -> Result<(), AtomiError> {
        let t = self.forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
            LinearStepperCommand::WaitIdle))).await;
        t.map(|_| ())
    }

    // TODO(lv): create an executor, put uart_comm into it. Then we don't need to pass uart_comm param.
    pub async fn execute_all_commands(&mut self) -> Result<(), AtomiError> {
        // TODO(lv): expect the response?
        let _ = self.forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::MoveTo {
            position: 0,
        }))).await?;
        self.wait_for_linear_stepper_available().await?;

        let _ = self.forward(AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::MoveTo {
            position: 0,
        }))).await?;

        let _ = self.forward(AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetPresserRotation { speed: 200 },
        ))).await?;

        let _ = self.forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::MoveTo {
            position: 200,
        }))).await?;
        self.wait_for_linear_stepper_available().await?;

        let _ = self.forward(AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetConveyorBeltRotation { speed: 200 },
        ))).await?;

        let _ = self.forward(AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
            idx: 0,
            speed: 1000,
        }))).await?;

        let _ = self.forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::MoveTo {
            position: 0,
        }))).await?;
        self.wait_for_linear_stepper_available().await?;

        let _ = self.forward(AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetConveyorBeltRotation { speed: 0 },
        ))).await?;

        let _ = self.forward(AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
            idx: 0,
            speed: 0,
        }))).await?;

        Ok(())
    }
}

pub async fn process_mc_one_full_run(mut executor: McSystemExecutor) {
    info!("process_mc_one_full_run() 0");
    let mq_in = system_executor_input_mq();
    let mq_out = system_executor_output_mq();
    loop {
        Delay::new(1.millis()).await;
        let _ = match mq_in.dequeue() {
            Some(McSystemExecutorCmd::ExecuteOneFullRun) => {
                let _ = executor.execute_all_commands().await;
                mq_out.enqueue(McSystemExecutorResponse::FinishedOneFullRun)
            }
            Some(McSystemExecutorCmd::ForwardRequest(msg)) => {
                let res = executor.forward(msg).await;
                let wrapped_msg = wrap_result_into_proto(res);
                mq_out.enqueue(McSystemExecutorResponse::ForwardResponse(wrapped_msg))
            }
            _ => continue,
        };
    }
}

// TODO(zephyr): put this into message_queue.rs
pub async fn wait_for_forward_dequeue() -> Result<AtomiProto, AtomiError> {
    let unit_delay = 10;
    let loop_times = 2_000 / unit_delay;
    for _ in 0..loop_times {
        Delay::new(unit_delay.millis()).wait();
        if let Some(McSystemExecutorResponse::ForwardResponse(resp))
            = system_executor_output_mq().dequeue() {
            info!("wait_for_forward_dequeue() 5: got response: {}", resp);
            return Ok(resp)
        }
    }
    // timeout
    Err(AtomiError::McForwardTimeout)
}