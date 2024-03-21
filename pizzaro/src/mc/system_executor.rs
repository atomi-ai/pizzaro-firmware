use cortex_m::asm::delay;
use defmt::{error, info, warn};
use fugit::ExtU64;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{
    wrap_result_into_proto, AtomiProto, DispenserCommand, HpdCommand, LinearBullCommand,
    LinearStepperCommand, McCommand, McSystemExecutorCmd, McSystemExecutorResponse, MmdCommand,
    PeristalticPumpCommand, RotationStepperCommand,
};
use generic::mmd_status::MmdStatus;

use crate::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use crate::common::global_timer::Delay;
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
use crate::common::uart_comm::UartComm;
use crate::mc::{UartDirType, UartType};

const PP_ON_SPEED: i32 = 300;
const PP_OFF_SPEED: i32 = 0;
const PR_ON_SPEED: i32 = 300;
const PR_OFF_SPEED: i32 = 0;
const DISPENSER_ON_SPEED: i32 = -1000;
const DISPENSER_OFF_SPEED: i32 = 0;
const BELT_ON_SPEED: i32 = 290;
const BELT_OFF_SPEED: i32 = 0;

static mut SYSTEM_EXECUTOR_INPUT_MQ_ONCE: Once<MessageQueueWrapper<AtomiProto>> = Once::new();
static mut SYSTEM_EXECUTOR_OUTPUT_MQ_ONCE: Once<MessageQueueWrapper<McSystemExecutorResponse>> =
    Once::new();
pub fn system_executor_input_mq() -> &'static mut MessageQueueWrapper<AtomiProto> {
    unsafe { SYSTEM_EXECUTOR_INPUT_MQ_ONCE.get_mut() }
}
pub fn system_executor_output_mq() -> &'static mut MessageQueueWrapper<McSystemExecutorResponse> {
    unsafe { SYSTEM_EXECUTOR_OUTPUT_MQ_ONCE.get_mut() }
}

pub struct McSystemExecutor {
    uart: UartType,
    uart_dir: Option<UartDirType>,
}

impl McSystemExecutor {
    pub fn new(uart: UartType, uart_dir: Option<UartDirType>) -> Self {
        Self { uart, uart_dir }
    }

    fn get_uart_comm(&mut self) -> UartComm<'_, UartDirType, UartType> {
        UartComm::new(
            &mut self.uart,
            &mut self.uart_dir,
            UART_EXPECTED_RESPONSE_LENGTH,
        )
    }

    pub async fn forward(&mut self, msg: AtomiProto) -> Result<AtomiProto, AtomiError> {
        info!("Forward msg: {}", msg);
        let mut uart_comm = self.get_uart_comm();
        if let Err(e) = uart_comm.send(msg) {
            error!("Error in sending command, err: {:?}", e);
            return Err(AtomiError::UartWriteError);
        }

        // 异步读取响应长度 or timeout
        uart_comm.recv_timeout::<AtomiProto>(500.millis()).await
    }

    async fn wait_for_linear_stepper_available(&mut self) -> Result<(), AtomiError> {
        loop {
            let t = self
                .forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                    LinearStepperCommand::WaitIdle,
                )))
                .await;
            match t {
                Ok(AtomiProto::AtomiError(AtomiError::MmdUnavailable(MmdStatus::Unavailable))) => {
                    // info!("unavailable, retry...");
                    Delay::new(500.millis()).await;
                    continue;
                }
                Ok(_) => {
                    // info!("ok: {}", r);
                    return t.map(|_| ());
                }
                Err(_) => {
                    // info!("errors: {}", e);
                    return t.map(|_| ());
                }
            }
        }
    }

    async fn wait_for_linear_bull_available(&mut self) -> Result<(), AtomiError> {
        loop {
            let t = self
                .forward(AtomiProto::Hpd(HpdCommand::HpdLinearBull(
                    LinearBullCommand::WaitIdle,
                )))
                .await;
            match t {
                Ok(AtomiProto::AtomiError(AtomiError::HpdUnavailable)) => {
                    // info!("unavailable, retry...");
                    Delay::new(500.millis()).await;
                    continue;
                }
                Ok(_) => {
                    // info!("ok: {}", r);
                    return t.map(|_| ());
                }
                Err(_) => {
                    // info!("errors: {}", e);
                    return t.map(|_| ());
                }
            }
        }
    }

    // TODO(lv): create an executor, put uart_comm into it. Then we don't need to pass uart_comm param.
    pub async fn execute_all_commands(&mut self) -> Result<(), AtomiError> {
        // TODO(lv): expect the response?

        // 伸缩传送带归位
        let _ = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                LinearStepperCommand::MoveTo {
                    position: 0,
                    speed: 0,
                },
            )))
            .await?;
        self.wait_for_linear_stepper_available().await?;

        //  压饼活塞归位
        let _ = self
            .forward(AtomiProto::Hpd(HpdCommand::HpdLinearBull(
                LinearBullCommand::MoveTo { position: 0 },
            )))
            .await?;
        self.wait_for_linear_bull_available().await?;

        // 压饼平台旋转
        let _ = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
                RotationStepperCommand::SetPresserRotation { speed: 200 },
            )))
            .await?;

        // 传送带伸出
        let _ = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                LinearStepperCommand::MoveTo {
                    position: 510,
                    speed: 400,
                },
            )))
            .await?;
        self.wait_for_linear_stepper_available().await?;

        // 传送带旋转
        let _ = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
                RotationStepperCommand::SetConveyorBeltRotation { speed: 200 },
            )))
            .await?;

        // 起司料斗旋转
        let _ = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdDisperser(
                DispenserCommand::SetRotation {
                    idx: 0,
                    speed: 1000,
                },
            )))
            .await?;

        // 传送带回收
        let _ = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                LinearStepperCommand::MoveTo {
                    position: 0,
                    speed: 500,
                },
            )))
            .await?;
        self.wait_for_linear_stepper_available().await?;

        // 传送带停止
        let _ = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
                RotationStepperCommand::SetConveyorBeltRotation { speed: 0 },
            )))
            .await?;

        // 平台停止旋转
        let _ = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdDisperser(
                DispenserCommand::SetRotation { idx: 0, speed: 0 },
            )))
            .await?;

        Ok(())
    }

    async fn mmd_linear_stepper_home(&mut self) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                LinearStepperCommand::Home,
            )))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn hpd_linear_bull_home(&mut self) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Hpd(HpdCommand::HpdLinearBull(
                LinearBullCommand::Home,
            )))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn mmd_pr(&mut self, speed: i32) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
                RotationStepperCommand::SetPresserRotation { speed },
            )))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn mmd_pr_off(&mut self) -> Result<(), AtomiError> {
        self.mmd_pr(PR_OFF_SPEED).await
    }

    async fn mmd_pr_on(&mut self) -> Result<(), AtomiError> {
        self.mmd_pr(PR_ON_SPEED).await
    }

    async fn mmd_pp(&mut self, speed: i32) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdPeristalticPump(
                PeristalticPumpCommand::SetRotation { speed },
            )))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn mmd_pp_off(&mut self) -> Result<(), AtomiError> {
        self.mmd_pp(PP_OFF_SPEED).await
    }

    async fn mmd_pp_on(&mut self) -> Result<(), AtomiError> {
        self.mmd_pp(PP_ON_SPEED).await
    }

    async fn mmd_dispenser(&mut self, idx: usize, speed: i32) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdDisperser(
                DispenserCommand::SetRotation { idx, speed },
            )))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn mmd_dispenser_off(&mut self, idx: usize) -> Result<(), AtomiError> {
        self.mmd_dispenser(idx, DISPENSER_OFF_SPEED).await
    }

    async fn mmd_dispenser_on(&mut self, idx: usize) -> Result<(), AtomiError> {
        self.mmd_dispenser(idx, DISPENSER_ON_SPEED).await
    }

    async fn mmd_belt(&mut self, speed: i32) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
                RotationStepperCommand::SetConveyorBeltRotation { speed },
            )))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn mmd_belt_off(&mut self) -> Result<(), AtomiError> {
        self.mmd_belt(BELT_OFF_SPEED).await
    }

    async fn mmd_belt_on(&mut self) -> Result<(), AtomiError> {
        self.mmd_belt(BELT_ON_SPEED).await
    }

    pub async fn system_init(&mut self) -> Result<(), AtomiError> {
        // Init the system
        self.mmd_linear_stepper_home().await?;
        self.hpd_linear_bull_home().await?;
        self.mmd_pr_off().await?; // self.pr_set(off)
        self.mmd_pp_off().await?; // self.pp_set(off)
        self.mmd_dispenser_off(0).await?; // self.dispenser(0, off)
        self.mmd_belt_off().await?; // self.belt_set(off)
        self.wait_for_linear_stepper_available().await?;
        self.wait_for_linear_bull_available().await?;
        Ok(())
    }

    pub async fn make_one_pizza(&mut self) -> Result<(), AtomiError> {
        // 压面团
        self.hpd_move_to(52700).await?;
        //self.hpd_move_to(22000).await?;
        self.wait_for_linear_bull_available().await?;
        Delay::new(3.secs()).await;
        self.hpd_move_to(22000).await?;
        self.wait_for_linear_bull_available().await?;
        // 挤番茄酱
        self.mmd_pr(300).await?;
        self.mmd_pp(-300).await?;

        self.mmd_move_to(210, 400).await?;
        self.mmd_pr(100).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(4.secs()).await;

        self.mmd_move_to(310, 400).await?;
        self.mmd_pr(150).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(3.secs()).await;

        self.mmd_move_to(410, 400).await?;
        self.mmd_pr(250).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(1500.millis()).await;

        self.mmd_move_to(510, 400).await?;
        self.mmd_pr(350).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(500.millis()).await;

        self.mmd_pp_off().await?;
        // 上cheese
        self.mmd_dispenser_on(0).await?;
        self.mmd_belt_on().await?;
        Delay::new(1500.millis()).await;

        self.mmd_pr(350).await?;
        self.mmd_move_to(410, 400).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(1.secs()).await;

        self.mmd_pr(300).await?;
        self.mmd_move_to(310, 400).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(1.secs()).await;

        self.mmd_pr(250).await?;
        self.mmd_move_to(210, 400).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(1.secs()).await;

        self.mmd_pr(200).await?;
        self.mmd_move_to(110, 400).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(1.secs()).await;

        self.mmd_pr(150).await?;
        self.mmd_move_to(0, 400).await?;
        self.wait_for_linear_stepper_available().await?;

        self.mmd_dispenser_off(0).await?;
        Delay::new(2.secs()).await;

        self.mmd_belt_off().await?;
        self.mmd_pr_off().await?;
        Ok(())
    }

    pub async fn hpd_move_to(&mut self, position: i32) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Hpd(HpdCommand::HpdLinearBull(
                LinearBullCommand::MoveTo { position },
            )))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    pub async fn mmd_move_to(&mut self, position: i32, speed: u32) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                LinearStepperCommand::MoveTo { position, speed },
            )))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }
}

fn expect_result(_actual: AtomiProto, _expected: AtomiProto) -> Result<(), AtomiError> {
    // TODO(zephyr): Figure a way to expect the correct result.
    Ok(())
}

fn error_or_done(
    res: Result<(), AtomiError>,
    mq_out: &mut MessageQueueWrapper<McSystemExecutorResponse>,
) {
    info!("Got result: {}", res);
    match res {
        Ok(_) => mq_out.enqueue(McSystemExecutorResponse::Done),
        Err(err) => mq_out.enqueue(McSystemExecutorResponse::Error(err)),
    }
}

pub async fn process_executor_requests(mut executor: McSystemExecutor) {
    info!("process_mc_one_full_run() 0");
    let mq_in = system_executor_input_mq();
    let mq_out = system_executor_output_mq();
    loop {
        Delay::new(1.millis()).await;
        let _ = match mq_in.dequeue() {
            Some(AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::InitSystem))) => {
                info!("Start to init the system");
                error_or_done(executor.system_init().await, mq_out);
                info!("Finish the system init");
            }
            Some(AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::MakePizza))) => {
                info!("Start to make a pizza");
                error_or_done(executor.make_one_pizza().await, mq_out);
                info!("Done on making a pizza");
            }
            Some(AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::ExecuteOneFullRun))) => {
                info!("Execute one full run");
                error_or_done(executor.make_one_pizza().await, mq_out);
                info!("Done on executing one full run");
            }
            Some(msg) => {
                let res = executor.forward(msg).await;
                let wrapped_msg = wrap_result_into_proto(res);
                mq_out.enqueue(McSystemExecutorResponse::ForwardResponse(wrapped_msg))
            }
            _ => continue,
        };
    }
}

// TODO(zephyr): put this into message_queue.rs
pub fn wait_for_forward_dequeue() -> Result<AtomiProto, AtomiError> {
    let unit_delay = 10;
    let loop_times = 2_000 / unit_delay;
    for _ in 0..loop_times {
        if let Some(McSystemExecutorResponse::ForwardResponse(resp)) =
            system_executor_output_mq().dequeue()
        {
            info!("wait_for_forward_dequeue() 5: got response: {}", resp);
            return Ok(resp);
        }
        delay(100_000);
    }
    warn!(
        "Not getting correct forward response, loop_times = {}",
        loop_times
    );
    // timeout
    Err(AtomiError::McForwardTimeout)
}
