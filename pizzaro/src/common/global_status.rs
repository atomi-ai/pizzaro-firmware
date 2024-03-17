extern crate alloc;

use alloc::vec;
use alloc::vec::Vec;
use core::cell::RefCell;

use critical_section::Mutex;
use defmt::info;

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum FutureStatus {
    Idle,
    Running,
    Completed,

    // Homing Status
    HomingStage1,
    HomingStage2,
    HomingStage3,
    HomingDone,

    // MMD status
    MmdAvailable,
    MmdBusy,

    // HPD status
    HpdAvailable,
    HpdBusy,
    // More future specific status
}

// Only add futures need to check status.
pub enum FutureType {
    Main, // Reserved for a future that handle everything.
    StepperHoming,
    Mmd,
    Hpd,
    End,
}

impl FutureType {
    pub fn index(&self) -> usize {
        match self {
            FutureType::Main => 0,
            FutureType::StepperHoming => 1,
            FutureType::Mmd => 2,
            FutureType::Hpd => 3,

            // End need to be the last index + 1.
            FutureType::End => 4,
        }
    }
}

static GLOBAL_STATUS: Mutex<RefCell<Vec<FutureStatus>>> = Mutex::new(RefCell::new(Vec::new()));

pub fn initialize_status() {
    critical_section::with(|cs| {
        let mut status = GLOBAL_STATUS.borrow(cs).borrow_mut();
        *status = vec![FutureStatus::Idle; FutureType::End.index()];
    });
}

#[allow(dead_code)]
fn log_global_status() {
    critical_section::with(|cs| {
        if let Ok(statuses) = GLOBAL_STATUS.borrow(cs).try_borrow() {
            info!(
                "global_status::log_global_status(): {:?}",
                statuses.as_slice()
            );
        } else {
            info!("global_status::Failed to borrow GLOBAL_STATUS");
        }
    });
}

pub fn set_status(ft: FutureType, new_status: FutureStatus) -> Option<FutureStatus> {
    critical_section::with(|cs| {
        if let Some(status) = GLOBAL_STATUS.borrow(cs).borrow_mut().get_mut(ft.index()) {
            let old_status = *status;
            *status = new_status;
            Some(old_status)
        } else {
            None
        }
    })
    // info!("global_status::set_status({}, {})", index, new_status);
    // log_global_status();
}

pub fn get_status(ft: FutureType) -> Option<FutureStatus> {
    critical_section::with(|cs| GLOBAL_STATUS.borrow(cs).borrow().get(ft.index()).copied())
    // info!("global_status::get_status({}) -> {}, copied: {}", index, res, t);
}
