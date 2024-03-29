use generic::{
    atomi_error::AtomiError,
    atomi_proto::{PeristalticPumpCommand, PeristalticPumpResponse},
};

use crate::bsp::MmdPeristalicPumpMotorType;

pub struct BrushMotorProcessor {
    peristaltic_pump_motor: MmdPeristalicPumpMotorType,
}

impl BrushMotorProcessor {
    pub fn new(mut peristaltic_pump_motor: MmdPeristalicPumpMotorType) -> Self {
        peristaltic_pump_motor.start_pwm_motor().unwrap();
        Self {
            peristaltic_pump_motor,
        }
    }

    pub fn set_peristaltic_pump_speed(&mut self, speed: i32) {
        self.peristaltic_pump_motor
            .apply_speed(speed as f32 / 1000.0);
    }

    pub fn process(
        &mut self,
        cmd: PeristalticPumpCommand,
    ) -> Result<PeristalticPumpResponse, AtomiError> {
        match cmd {
            PeristalticPumpCommand::SetRotation { speed } => {
                self.set_peristaltic_pump_speed(speed);
                Ok(PeristalticPumpResponse::Done)
            }
        }
    }
}
