use defmt::{debug, info};
use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin};
use embedded_hal::serial::{Read, Write};
use fugit::ExtU64;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{StepperCommand, StepperResponse, TriggerStatusResponse};

use crate::common::global_timer::{AsyncDelay, Delay};
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
use crate::dtu::tmc_stepper::TmcStepper;

static mut DTU_STEPPER_INPUT_MQ_ONCE: Once<MessageQueueWrapper<StepperCommand>> = Once::new();
static mut DTU_STEPPER_OUTPUT_MQ_ONCE: Once<MessageQueueWrapper<StepperResponse>> = Once::new();

pub fn dtu_stepper_input_mq() -> &'static mut MessageQueueWrapper<StepperCommand> {
    unsafe { DTU_STEPPER_INPUT_MQ_ONCE.get_mut() }
}

pub fn dtu_stepper_output_mq() -> &'static mut MessageQueueWrapper<StepperResponse> {
    unsafe { DTU_STEPPER_OUTPUT_MQ_ONCE.get_mut() }
}

pub struct TmcStepperProcessor<
    IP1: InputPin,
    IP2: InputPin,
    OP1: StatefulOutputPin,
    OP2: OutputPin,
    OP3: OutputPin,
    D: AsyncDelay,
    U: Write<u8>,
    X: Read<u8>,
> {
    stepper: TmcStepper<IP1, IP2, OP1, OP2, OP3, D, U, X>,
}

impl<IP1, IP2, OP1, OP2, OP3, D, U, X> TmcStepperProcessor<IP1, IP2, OP1, OP2, OP3, D, U, X>
where
    IP1: InputPin,
    IP2: InputPin,
    OP1: StatefulOutputPin,
    OP2: OutputPin,
    OP3: OutputPin,
    D: AsyncDelay,
    U: Write<u8>,
    X: Read<u8>,
{
    pub fn new(stepper: TmcStepper<IP1, IP2, OP1, OP2, OP3, D, U, X>) -> Self {
        Self { stepper }
    }

    pub async fn process_stepper_request<'a>(
        &mut self,
        msg: StepperCommand,
    ) -> Result<i32, AtomiError> {
        match msg {
            StepperCommand::Home => self.stepper.home().await,
            StepperCommand::MoveTo { position, speed } => {
                self.stepper.move_to(position, speed).await
            }
            StepperCommand::MoveToRelative { steps, speed } => {
                self.stepper.move_to_relative(steps, speed).await
            }
            StepperCommand::MoveToRelativeForce { steps, speed } => {
                self.stepper.move_to_relative_by_force(steps, speed).await
            }
            StepperCommand::Off => {
                self.stepper.disable().unwrap();
                Ok(0)
            }
            StepperCommand::GetTriggerStatus => {
                // 在调用这个函数之前已经独立处理了，这里不应该进入这个分支，所以不需要做任何事情
                Ok(0)
            }
            StepperCommand::WaitIdle => {
                while !self.stepper.is_idle() {
                    let _ = Delay::new(300.millis()).await;
                    info!("wait idle...")
                }
                Ok(0)
            }
            StepperCommand::DummyWait { seconds } => {
                // Testing only.
                let _ = Delay::new((seconds as u64).secs()).await;
                Ok(0)
            }
        }
    }
}

pub async fn process_dtu_stepper_message<
    IP1: InputPin,
    IP2: InputPin,
    OP1: StatefulOutputPin,
    OP2: OutputPin,
    OP3: OutputPin,
    D: AsyncDelay,
    U: Write<u8>,
    X: Read<u8>,
>(
    mut processor: TmcStepperProcessor<IP1, IP2, OP1, OP2, OP3, D, U, X>,
) {
    debug!("process_dtu_stepper_message() 0");
    let mq_in = dtu_stepper_input_mq();
    let mq_out = dtu_stepper_output_mq();
    loop {
        if let Some(msg) = mq_in.dequeue() {
            debug!("process_dtu_stepper_message() 3.1: process msg {}", msg);
            if msg == StepperCommand::GetTriggerStatus {
                let (l, r) = processor.stepper.get_limit_status();
                mq_out.enqueue(StepperResponse::TriggerStatus(TriggerStatusResponse {
                    left: l,
                    right: r,
                }));
            }
            let res = match processor.process_stepper_request(msg).await {
                Ok(_) => StepperResponse::Done,
                Err(err) => StepperResponse::Error(err),
            };
            debug!("process_dtu_stepper_message() 3.3: done");
            mq_out.enqueue(res);
        }
        Delay::new(1.millis()).await;
    }
}
