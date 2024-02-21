use rp2040_hal::gpio::bank0::{Gpio11, Gpio18, Gpio22, Gpio26, Gpio27, Gpio4, Gpio5};
use rp2040_hal::gpio::{FunctionSio, FunctionUart, Pin, PullDown, SioInput, SioOutput};
use rp2040_hal::pac::UART1;
use rp2040_hal::uart::{Enabled, UartPeripheral};

use common::global_status::{FutureStatus, FutureType, set_status};
use common::global_timer::DelayCreator;
use generic::atomi_error::AtomiError;
use generic::atomi_proto::{AtomiProto, MmdCommand};

use crate::mmd::linear_stepper::LinearStepper;
use crate::uart_comm::UartComm;

pub type EnablePinType = Pin<Gpio22, FunctionSio<SioOutput>, PullDown>;
pub type DirPinType = Pin<Gpio18, FunctionSio<SioOutput>, PullDown>;
pub type StepPinType = Pin<Gpio11, FunctionSio<SioOutput>, PullDown>;
pub type LimitLeftPinType = Pin<Gpio26, FunctionSio<SioInput>, PullDown>;
pub type LimitRightPinType = Pin<Gpio27, FunctionSio<SioInput>, PullDown>;
pub type LinearStepperType = LinearStepper<LimitLeftPinType, LimitRightPinType, EnablePinType, DirPinType, StepPinType, DelayCreator>;

pub type UartType = UartPeripheral<Enabled, UART1, (
    Pin<Gpio4, FunctionUart, PullDown>, Pin<Gpio5, FunctionUart, PullDown>)>;

pub struct MmdProcessor {
    linear_stepper: LinearStepperType,
}

impl MmdProcessor {
    pub fn new(linear_stepper: LinearStepperType) -> Self {
        Self { linear_stepper }
    }
    pub async fn process_mmd_message<'a>(&mut self, uart_comm: &mut UartComm<'a, UartType>, msg: MmdCommand) -> Result<(), AtomiError>{
        set_status(FutureType::Mmd, FutureStatus::MmdBusy);
        match msg {
            MmdCommand::MmdPing =>
                uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdPong))?,
            MmdCommand::MmdHome => {
                uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdInHoming))?;
                self.home().await.map(|_| ())?
            },
            MmdCommand::MmdMoveTo { position } => {
                uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdOnMove))?;
                self.move_to(position).await.map(|_| ())?
            },
            MmdCommand::MmdMoveToRelative { steps } => {
                uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdOnMove))?;
                self.move_to_relative(steps).await.map(|_| ())?
            },

            _ => Err(AtomiError::UnaccepableCommand)?,
        };
        set_status(FutureType::Mmd, FutureStatus::MmdAvailable);
        Ok(())
    }

    pub async fn home(&mut self) -> Result<i32, AtomiError> {
        self.linear_stepper.home().await
    }

    pub async fn move_to_relative(&mut self, steps: i32) -> Result<i32, AtomiError> {
        self.linear_stepper.move_to_relative(steps).await
    }

    pub async fn move_to(&mut self, position: i32) -> Result<i32, AtomiError> {
        self.linear_stepper.move_to(position).await
    }
}
