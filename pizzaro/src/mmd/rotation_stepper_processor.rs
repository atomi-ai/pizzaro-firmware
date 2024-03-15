use crate::common::message_queue::MessageQueueInterface;
use defmt::info;
use fugit::ExtU64;
use generic::{
    atomi_error::AtomiError,
    atomi_proto::{RotationStepperCommand, RotationStepperResponse},
};

use crate::{
    bsp::{ConveyorBeltRotationMotorType, PresserRotationMotorType},
    common::{global_timer::Delay, message_queue::MessageQueueWrapper, once::Once},
};

static mut ROTATION_STEPPER_INPUT_MQ_ONCE: Once<MessageQueueWrapper<RotationStepperCommand>> =
    Once::new();
static mut ROTATION_STEPPER_OUTPUT_MQ_ONCE: Once<MessageQueueWrapper<RotationStepperResponse>> =
    Once::new();
pub fn rotation_stepper_input_mq() -> &'static mut MessageQueueWrapper<RotationStepperCommand> {
    unsafe { ROTATION_STEPPER_INPUT_MQ_ONCE.get_mut() }
}
pub fn rotation_stepper_output_mq() -> &'static mut MessageQueueWrapper<RotationStepperResponse> {
    unsafe { ROTATION_STEPPER_OUTPUT_MQ_ONCE.get_mut() }
}

pub struct RotationStepperProcessor {
    conveyor_rotation_stepper: ConveyorBeltRotationMotorType,
    presser_rotation_stepper: PresserRotationMotorType,
}

impl RotationStepperProcessor {
    pub fn new(
        conveyor_rotation_stepper: ConveyorBeltRotationMotorType,
        presser_rotation_stepper: PresserRotationMotorType,
    ) -> Self {
        Self {
            conveyor_rotation_stepper,
            presser_rotation_stepper,
        }
    }

    pub async fn process_rotation_stepper_request(
        &mut self,
        msg: RotationStepperCommand,
    ) -> Result<(), AtomiError> {
        match msg {
            RotationStepperCommand::SetConveyorBeltRotation { speed } => {
                self.conveyor_rotation_stepper.set_direction(speed > 0)?;
                self.conveyor_rotation_stepper
                    .set_speed(speed.unsigned_abs());
                Ok(())
            }
            RotationStepperCommand::SetPresserRotation { speed } => {
                self.presser_rotation_stepper.set_direction(speed > 0)?;
                self.presser_rotation_stepper
                    .set_speed(speed.unsigned_abs());
                Ok(())
            }
        }
    }
}

pub async fn process_mmd_rotation_stepper_message(mut processor: RotationStepperProcessor) {
    info!("process_mmd_rotation_stepper_message() 0");
    let mq_in = rotation_stepper_input_mq();
    let mq_out = rotation_stepper_output_mq();
    loop {
        if let Some(msg) = mq_in.dequeue() {
            info!(
                "process_mmd_rotation_stepper_message() 3.1: process msg {}",
                msg
            );
            let res = match processor.process_rotation_stepper_request(msg).await {
                Ok(_) => RotationStepperResponse::Done,
                Err(err) => RotationStepperResponse::Error(err),
            };
            info!("process_mmd_rotation_stepper_message() 3.3: done");
            mq_out.enqueue(res);
        }
        Delay::new(1.millis()).await;
    }
}
