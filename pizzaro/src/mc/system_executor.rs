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

use crate::common::consts::{
    BELT_OFF_SPEED, BELT_ON_SPEED, DISPENSER_OFF_SPEED, DISPENSER_ON_SPEED, PP_OFF_SPEED,
    PP_ON_SPEED, PR_OFF_SPEED, PR_ON_SPEED, UART_EXPECTED_RESPONSE_LENGTH,
};
use crate::common::global_timer::Delay;
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
use crate::common::uart_comm::UartComm;
use crate::mc::{UartDirType, UartType};

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
        UartComm::new(&mut self.uart, &mut self.uart_dir, UART_EXPECTED_RESPONSE_LENGTH)
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
                .forward(AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::WaitIdle)))
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

    async fn mmd_linear_stepper_home(&mut self) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::Home)))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn hpd_linear_bull_home(&mut self) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::Home)))
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

    async fn _mmd_pr_on(&mut self) -> Result<(), AtomiError> {
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

    async fn _mmd_pp_on(&mut self) -> Result<(), AtomiError> {
        self.mmd_pp(PP_ON_SPEED).await
    }

    async fn mmd_dispenser(&mut self, idx: usize, speed: i32) -> Result<(), AtomiError> {
        let res =
            self.forward(AtomiProto::Mmd(MmdCommand::MmdDisperser(
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
        info!("set belt spd:{}", speed);
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
        self.mmd_belt(BELT_ON_SPEED / 4).await?;
        Delay::new(100u64.millis()).await;
        self.mmd_belt(BELT_ON_SPEED / 2).await?;
        Delay::new(100u64.millis()).await;
        self.mmd_belt(BELT_ON_SPEED * 3 / 4).await?;
        Delay::new(100u64.millis()).await;
        self.mmd_belt(BELT_ON_SPEED).await?;
        Delay::new(100u64.millis()).await;
        Ok(())
    }

    async fn _hpd_move_to(&mut self, position: i32) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::MoveTo {
                position,
            })))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn mmd_move_to(&mut self, position: i32, speed: u32) -> Result<(), AtomiError> {
        let res =
            self.forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(
                LinearStepperCommand::MoveTo { position, speed },
            )))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn hpd_stop(&mut self) -> Result<(), AtomiError> {
        let res = self.forward(AtomiProto::Hpd(HpdCommand::HpdStop)).await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn mmd_stop(&mut self) -> Result<(), AtomiError> {
        let res = self.forward(AtomiProto::Mmd(MmdCommand::MmdStop)).await?;
        expect_result(res, AtomiProto::Unknown)
    }

    pub async fn system_stop(&mut self) -> Result<(), AtomiError> {
        self.hpd_stop().await?;
        self.mmd_stop().await
        // TODO(zephyr): Let's try the join version below in the future.
        // let f1 = self.hpd_stop();
        // let f2 = self.mmd_stop();
        // let (res1, res2) = join(f1, f2).await;
        // match (res1, res2) {
        //     (Ok(_), Ok(_)) => Ok(()),
        //     (Err(err), _) => Err(err),
        //     (_, Err(err)) => Err(err),
        // }
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

    pub async fn squeeze_ketchup(&mut self) -> Result<(), AtomiError> {
        // 挤番茄酱
        // init
        self.mmd_pr(300).await?;
        self.mmd_pp(-300).await?;

        self.mmd_move_to(200, 500).await?;
        self.mmd_pr(218).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(6329.millis()).await;

        self.mmd_move_to(303, 500).await?;
        self.mmd_pr(277).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(4973.millis()).await;

        self.mmd_move_to(406, 500).await?;
        self.mmd_pr(381).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(3617.millis()).await;

        self.mmd_move_to(510, 500).await?;
        self.mmd_pr(611).await?;
        self.wait_for_linear_stepper_available().await?;
        Delay::new(2260.millis()).await;

        // finish
        self.mmd_pp_off().await?;
        Ok(())
    }

    pub async fn sprinkle_cheese(&mut self) -> Result<(), AtomiError> {
        let belt_spd_on_move = 0;
        let spd_amp1 = 1.0;
        let spd_amp2 = 1.0;
        let spd_amp3 = 1.0;
        let delay_acc = 1.0;
        let delay_revert = 800_f32;
        // 启动传送带和起司，等待1.5秒让起司掉落到传送带上
        // init-temp
        // self.mmd_move_to(510, 500).await?;
        // self.mmd_pr(759).await?;
        // self.wait_for_linear_stepper_available().await?;

        // 传送带伸出去
        self.mmd_move_to(510, 400).await?;
        // 转盘启动
        self.mmd_pr((459_f32 * spd_amp1) as i32).await?;
        // 等待传送带运行就位
        self.wait_for_linear_stepper_available().await?;
        // 转盘达到正式速度
        self.mmd_pr((759_f32 * spd_amp1) as i32).await?;

        // init
        // 初始化，料斗启动，正转
        self.mmd_dispenser(0, DISPENSER_ON_SPEED).await?;
        // 传送带启动
        self.mmd_belt_on().await?;
        // 等待一小会儿，等起司走完传送带
        Delay::new(1500.millis()).await;

        //self.mmd_belt(BELT_ON_SPEED + belt_spd_on_move).await?;
        //self.mmd_move_to(510, 400).await?;
        //self.mmd_dispenser_off(0).await?;
        //self.mmd_belt(BELT_ON_SPEED).await?;
        //self.mmd_pr((759_f32 * spd_amp1) as i32).await?;
        //        Delay::new(((delay_revert) as u64).millis()).await;
        //self.wait_for_linear_stepper_available().await?;
        //self.mmd_dispenser(0, DISPENSER_ON_SPEED).await?;

        //Delay::new(1819.millis()).await;
        // 等待第一圈撒完
        Delay::new(((1819_f32 / delay_acc) as u64).millis()).await;

        // 传送带回撤，补偿回撤速度
        self.mmd_belt(BELT_ON_SPEED + belt_spd_on_move).await?;
        // 停止转盘
        self.mmd_pr(0).await?;
        // 停止料斗
        self.mmd_dispenser_off(0).await?;
        // 传送带回撤
        self.mmd_move_to(355, 400).await?;
        // 等待传送带回撤结束
        self.wait_for_linear_stepper_available().await?;
        // 重新启动传送带
        self.mmd_belt(BELT_ON_SPEED).await?;
        // 重新启动料斗，正转
        self.mmd_dispenser(0, DISPENSER_ON_SPEED).await?;
        // // 抵消反转造成的非连续性
        // Delay::new(((delay_revert) as u64).millis()).await;
        //        Delay::new(((delay_revert) as u64).millis()).await;
        // 启动转盘
        self.mmd_pr((421_f32 * spd_amp2) as i32).await?;
        // 等待撒完
        //Delay::new(((3275_f32 / delay_acc) as u64).millis()).await;
        Delay::new(((3275_f32 / delay_acc) as u64).millis()).await;

        // 传送带回撤，补偿回撤速度
        self.mmd_belt(BELT_ON_SPEED + belt_spd_on_move).await?;
        // 停止转盘
        self.mmd_pr(0).await?;
        // 停止料斗
        self.mmd_dispenser_off(0).await?;
        // 传送带回撤
        self.mmd_move_to(200, 400).await?;
        // 等待传送带回撤结束
        self.wait_for_linear_stepper_available().await?;
        // 停止传送带
        self.mmd_belt_off().await?;
        // 重新启动料斗，反转
        self.mmd_dispenser(0, -DISPENSER_ON_SPEED).await?;
        // 抵消反转造成的非连续性
        Delay::new(((delay_revert) as u64).millis()).await;
        //        Delay::new(((delay_revert) as u64).millis()).await;
        // 启动转盘
        self.mmd_pr((292_f32 * spd_amp3) as i32).await?;
        // 恢复传送带速度
        self.mmd_belt(BELT_ON_SPEED).await?;
        //等待撒完
        //Delay::new(((4730_f32 / delay_acc) as u64).millis()).await;
        Delay::new(((4230_f32 / delay_acc) as u64).millis()).await;

        // 下面要跳过一小块区域，这里容易产生堆积
        // 暂停传送带
        self.mmd_belt_off().await?;
        // 转盘提速
        self.mmd_pr(700).await?;
        // 等转过去一点再启动
        Delay::new(600_u64.millis()).await;
        // 转盘恢复之前的速度
        self.mmd_pr((292_f32 * spd_amp3) as i32).await?;
        // 恢复传送带速度
        self.mmd_belt(BELT_ON_SPEED).await?;

        // 等待传送带上剩余起司全部送出
        self.mmd_dispenser_off(0).await?;
        Delay::new(1800u64.millis()).await;
        Ok(())
    }

    pub async fn make_one_pizza(&mut self) -> Result<(), AtomiError> {
        // 压面团
        // self.hpd_move_to(52700).await?;
        // //self.hpd_move_to(22000).await?;
        // self.wait_for_linear_bull_available().await?;
        // Delay::new(3.secs()).await;
        // self.hpd_move_to(22000).await?;
        // self.wait_for_linear_bull_available().await?;

        // 涂番茄酱
        //        self.squeeze_ketchup().await?;
        // 撒起司
        self.sprinkle_cheese().await?;

        // 结束
        self.mmd_move_to(0, 500).await?;
        // self.hpd_move_to(0).await?;
        self.wait_for_linear_stepper_available().await?;
        self.mmd_belt_off().await?;
        self.mmd_pr_off().await?;
        Ok(())
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
        match mq_in.dequeue() {
            Some(AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::StopSystem))) => {
                info!("To stop the system");
                error_or_done(executor.system_stop().await, mq_out);
                info!("The system is stopped");
            }
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
    warn!("Not getting correct forward response, loop_times = {}", loop_times);
    // timeout
    Err(AtomiError::McForwardTimeout)
}
