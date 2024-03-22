use defmt::Format;
use generic::atomi_error::AtomiError;
use heapless::Vec;
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, PartialEq, Format, Debug, Serialize, Deserialize)]
pub enum LinearMotionState {
    IDLE,
    MOVING,
    HOMING,
}

#[derive(Clone)]
pub struct MotionState {
    state: Vec<LinearMotionState, 16>,
}

impl Format for MotionState {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "MotionState {{ state: [");

        for (i, s) in self.state.iter().enumerate() {
            if i != 0 {
                defmt::write!(fmt, ", ");
            }
            defmt::write!(fmt, "{:?}", s);
        }

        defmt::write!(fmt, "] }}");
    }
}

impl MotionState {
    pub fn new() -> Self {
        let state_array = [LinearMotionState::IDLE];

        Self { state: Vec::from_slice(&state_array).unwrap() }
    }

    pub fn push(&mut self, new_state: LinearMotionState) -> Result<(), AtomiError> {
        self.state.push(new_state).map_err(|_| AtomiError::StateOverflow)
    }

    pub fn pop(&mut self) -> Option<LinearMotionState> {
        self.state.pop()
    }

    pub fn is_idle(&self) -> bool {
        match self.state.last() {
            None => panic!("should never happen"),
            Some(e) => *e == LinearMotionState::IDLE,
        }
    }
}
