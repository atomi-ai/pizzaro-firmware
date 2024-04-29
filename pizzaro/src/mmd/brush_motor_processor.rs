use crate::bsp::board_mmd_release_sb::MmdPeristalicPumpMotorType;

pub struct MmdPeristalicPumpProcessor {
    peristaltic_pump_motor: MmdPeristalicPumpMotorType,
}

impl MmdPeristalicPumpProcessor {
    pub fn new(peristaltic_pump_motor: MmdPeristalicPumpMotorType) -> Self {
        Self { peristaltic_pump_motor }
    }

    pub fn set_speed(&mut self, speed: i32) {
        self.peristaltic_pump_motor.apply_speed(speed as f32 / 1000.0);
    }
}
