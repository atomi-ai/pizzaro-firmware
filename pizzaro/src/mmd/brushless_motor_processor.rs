use crate::bsp::board_mmd_release_sb::{MmdDisperser0MotorType, MmdDisperser1MotorType};
use generic::{
    atomi_error::AtomiError,
    atomi_proto::{DispenserCommand, DispenserResponse},
};

pub struct DispenserMotorProcessor {
    dispenser0_motor: MmdDisperser0MotorType,
    dispenser1_motor: MmdDisperser1MotorType,
}

impl DispenserMotorProcessor {
    pub fn new(
        dispenser0_motor: MmdDisperser0MotorType,
        dispenser1_motor: MmdDisperser1MotorType,
    ) -> Self {
        // dispenser0_motor.enable().unwrap();
        // dispenser1_motor.enable().unwrap();
        Self { dispenser0_motor, dispenser1_motor }
    }

    pub fn set_dispenser0_speed(&mut self, speed: i32) {
        self.dispenser0_motor.apply_speed(speed as f32 / 1000.0);
    }

    pub fn set_dispenser1_speed(&mut self, speed: i32) {
        self.dispenser1_motor.apply_speed(speed as f32 / 1000.0);
    }
    pub fn get_dispenser0_counter(&mut self) -> Result<DispenserResponse, AtomiError> {
        todo!();
        //Ok(DispenserResponse::Counter { idx: 0, value: 0 })
    }

    pub fn get_dispenser1_counter(&mut self) -> Result<DispenserResponse, AtomiError> {
        todo!();
        //Ok(DispenserResponse::Counter { idx: 1, value: 0 })
    }

    pub fn reset_dispenser0_counter(&mut self) {
        todo!()
    }

    pub fn reset_dispenser1_counter(&mut self) {
        todo!()
    }

    pub fn process(&mut self, cmd: DispenserCommand) -> Result<DispenserResponse, AtomiError> {
        match cmd {
            DispenserCommand::SetRotation { idx: 0, speed } => {
                self.set_dispenser0_speed(speed);
                Ok(DispenserResponse::Done { idx: 0 })
            }
            DispenserCommand::SetRotation { idx: 1, speed } => {
                self.set_dispenser1_speed(speed);
                Ok(DispenserResponse::Done { idx: 1 })
            }
            DispenserCommand::ResetCounter { idx: 0 } => {
                self.reset_dispenser0_counter();
                Ok(DispenserResponse::Done { idx: 0 })
            }
            DispenserCommand::ResetCounter { idx: 1 } => {
                self.reset_dispenser1_counter();
                Ok(DispenserResponse::Done { idx: 1 })
            }
            DispenserCommand::GetCounter { idx: 0 } => self.get_dispenser0_counter(),
            DispenserCommand::GetCounter { idx: 1 } => self.get_dispenser1_counter(),
            _ => Err(AtomiError::MmdUnknownDispenserIdx),
        }
    }
}
