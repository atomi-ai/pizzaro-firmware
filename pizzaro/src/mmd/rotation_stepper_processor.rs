use crate::bsp::MmdPresserMotorType;
use defmt::info;
use generic::{
    atomi_error::AtomiError,
    atomi_proto::{RotationStepperCommand},
};
use generic::atomi_proto::{AtomiProto, MmdCommand};

use crate::{
    bsp::ConveyorBeltRotationMotorType,
};

pub struct RotationStepperProcessor {
    conveyor_rotation_stepper: ConveyorBeltRotationMotorType,
    presser_rotation_stepper: MmdPresserMotorType,
}

impl RotationStepperProcessor {
    pub fn new(
        conveyor_rotation_stepper: ConveyorBeltRotationMotorType,
        presser_rotation_stepper: MmdPresserMotorType,
    ) -> Self {
        Self {
            conveyor_rotation_stepper,
            presser_rotation_stepper,
        }
    }

    pub fn set_conveyor_speed(&mut self, speed: i32) {
        self.conveyor_rotation_stepper.set_speed(speed)
    }

    pub fn set_presser_speed(&mut self, speed: i32) {
        self.presser_rotation_stepper.set_speed(speed)
    }


    pub fn process_rotation_stepper_request(
        &mut self,
        msg: RotationStepperCommand,
    ) -> Result<AtomiProto, AtomiError> {
        info!("process_rotation_stepper_request: {}", msg);
        match msg {
            RotationStepperCommand::SetConveyorBeltRotation { speed } => {
                info!("set conveyor belt rotation, spd:{}", speed);
                self.set_conveyor_speed(speed);
                Ok(AtomiProto::Mmd(MmdCommand::MmdAck))
            }
            RotationStepperCommand::SetPresserRotation { speed } => {
                self.set_presser_speed(speed);
                Ok(AtomiProto::Mmd(MmdCommand::MmdAck))
            }
        }
    }
}
