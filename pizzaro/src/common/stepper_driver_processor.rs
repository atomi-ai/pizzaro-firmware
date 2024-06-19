use crate::common::global_timer::{AsyncDelay, Delay};
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
use crate::common::stepper::classic_stepper_driver::StepperDriver;
use defmt::{debug, Debug2Format};
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use fugit::ExtU64;
use generic::atomi_error::AtomiError;
use generic::atomi_proto::{StepperDriverCommand, StepperDriverResponse};

static mut STEPPER_DRIVER_INPUT_MQ_ONCE: Once<MessageQueueWrapper<StepperDriverCommand>> =
    Once::new();
static mut STEPPER_DRIVER_OUTPUT_MQ_ONCE: Once<MessageQueueWrapper<StepperDriverResponse>> =
    Once::new();

pub fn stepper_driver_input_mq() -> &'static mut MessageQueueWrapper<StepperDriverCommand> {
    unsafe { STEPPER_DRIVER_INPUT_MQ_ONCE.get_mut() }
}

pub fn stepper_driver_output_mq() -> &'static mut MessageQueueWrapper<StepperDriverResponse> {
    unsafe { STEPPER_DRIVER_OUTPUT_MQ_ONCE.get_mut() }
}

pub struct StepperDriverProcessor<
    OP1: StatefulOutputPin,
    OP2: OutputPin,
    OP3: OutputPin,
    D: AsyncDelay,
> {
    stepper_driver: StepperDriver<OP1, OP2, OP3, D>,
}

impl<OP1, OP2, OP3, D> StepperDriverProcessor<OP1, OP2, OP3, D>
where
    OP1: StatefulOutputPin,
    OP2: OutputPin,
    OP3: OutputPin,
    D: AsyncDelay,
{
    pub fn new(stepper_driver: StepperDriver<OP1, OP2, OP3, D>) -> Self {
        Self { stepper_driver }
    }

    pub async fn process_stepper_driver_request<'a>(
        &mut self,
        msg: StepperDriverCommand,
    ) -> Result<(), AtomiError> {
        const STEPS_TO_ADJUST_SPEED: i32 = 1000;
        if let StepperDriverCommand::MoveToRelative { steps, speed } = msg {
            self.stepper_driver.ensure_enable()?;
            // self.stepper_driver.set_speed(speed);
            self.stepper_driver.set_direction(steps >= 0)?;
            for i in 0..steps.abs() {
                if i % STEPS_TO_ADJUST_SPEED == 0 {
                    let cur_spd = (i / STEPS_TO_ADJUST_SPEED + 1) as u32 * 1000;
                    if cur_spd <= speed {
                        debug!(
                            "process_stepper_driver_request() 3.3: adjust speed: {}, i = {}",
                            cur_spd, i
                        );
                        self.stepper_driver.set_speed(cur_spd);
                    }
                }
                self.stepper_driver.step().await?;
            }
        }

        Ok(())
    }
}

pub async fn process_stepper_driver_message<
    OP1: StatefulOutputPin,
    OP2: OutputPin,
    OP3: OutputPin,
    D: AsyncDelay,
>(
    mut processor: StepperDriverProcessor<OP1, OP2, OP3, D>,
) {
    debug!("process_stepper_driver_message() 0");
    let mq_in = stepper_driver_input_mq();
    let mq_out = stepper_driver_output_mq();
    loop {
        if let Some(msg) = mq_in.dequeue() {
            debug!("process_stepper_driver_message() 3.1: process msg {}", Debug2Format(&msg));
            let resp = match processor.process_stepper_driver_request(msg).await {
                Ok(_) => StepperDriverResponse::Done,
                Err(err) => StepperDriverResponse::Error(err),
            };
            debug!("process_stepper_driver_message() 3.3: done, resp: {}", Debug2Format(&resp));
            mq_out.enqueue(resp);
        }
        Delay::new(1.millis()).await;
    }
}
