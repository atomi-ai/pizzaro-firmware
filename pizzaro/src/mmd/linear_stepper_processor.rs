use defmt::info;
use fugit::ExtU64;
use generic::atomi_error::AtomiError;
use generic::atomi_proto::{LinearStepperCommand, LinearStepperResponse};
use crate::common::global_timer::Delay;
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
use crate::mmd::mmd_dispatcher::LinearStepperType;

static mut LINEAR_STEPPER_INPUT_MQ_ONCE: Once<MessageQueueWrapper<LinearStepperCommand>> = Once::new();
static mut LINEAR_STEPPER_OUTPUT_MQ_ONCE: Once<MessageQueueWrapper<LinearStepperResponse>> = Once::new();
pub fn linear_stepper_input_mq() -> &'static mut MessageQueueWrapper<LinearStepperCommand> {
    unsafe { LINEAR_STEPPER_INPUT_MQ_ONCE.get_mut() }
}
pub fn linear_stepper_output_mq() -> &'static mut MessageQueueWrapper<LinearStepperResponse> {
    unsafe { LINEAR_STEPPER_OUTPUT_MQ_ONCE.get_mut() }
}

pub struct LinearStepperProcessor {
    linear_stepper: LinearStepperType,
}

impl LinearStepperProcessor {
    pub fn new(linear_stepper: LinearStepperType) -> Self {
        Self { linear_stepper }
    }

    pub async fn process_linear_stepper_request<'a> (
        &mut self, msg: LinearStepperCommand) -> Result<i32, AtomiError> {
        match msg {
            LinearStepperCommand::Home => {
                self.linear_stepper.home().await
            }
            LinearStepperCommand::MoveTo { position } => {
                self.linear_stepper.move_to(position).await
            }
            LinearStepperCommand::MoveToRelative { steps } => {
                self.linear_stepper.move_to_relative(steps).await
            }
            LinearStepperCommand::DummyWait { seconds } => {
                // Testing only.
                let _ = Delay::new((seconds as u64).secs()).await;
                Ok(0)
            }
        }
    }
}

pub async fn process_mmd_linear_stepper_message(mut processor: LinearStepperProcessor) {
    info!("process_mmd_linear_stepper_message() 0");
    let mq_in = linear_stepper_input_mq();
    let mq_out = linear_stepper_output_mq();
    loop {
        if let Some(msg) = mq_in.dequeue() {
            info!("process_mmd_linear_stepper_message() 3.1: process msg {}", msg);
            let res = match processor.process_linear_stepper_request(msg).await {
                Ok(_) => LinearStepperResponse::Done,
                Err(err) => LinearStepperResponse::Error(err),
            };
            info!("process_mmd_linear_stepper_message() 3.3: done");
            mq_out.enqueue(res);
        }
        Delay::new(1.millis()).await;
    }
}
