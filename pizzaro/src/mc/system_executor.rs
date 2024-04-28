use cortex_m::asm::delay;
use defmt::{debug, error, info, warn};
use fugit::ExtU64;
use generic::atomi_error::AtomiError;
use generic::atomi_proto::{
    wrap_result_into_proto, AtomiProto, DispenserCommand, DtuCommand, HpdCommand,
    LinearBullCommand, McCommand, McSystemExecutorCmd, McSystemExecutorResponse, MmdCommand,
    PeristalticPumpCommand, RotationStepperCommand, StepperCommand,
};
use generic::mmd_status::MmdStatus;

use crate::common::consts::{
    BELT_OFF_SPEED, BELT_ON_SPEED, DISPENSER_OFF_SPEED, DISPENSER_ON_SPEED,
    LINEAR_BULL_MAX_PRESSURE, PP_OFF_SPEED, PP_ON_SPEED, PR_OFF_SPEED, PR_ON_SPEED,
    UART_EXPECTED_RESPONSE_LENGTH,
};
use crate::common::global_timer::Delay;
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;
use crate::common::uart_comm::UartComm;
use crate::common::weight_sensor::WeightSensors;
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
    weight_sensors: WeightSensors,
    weight_zero_values: Option<[u32; 4]>,
}

impl McSystemExecutor {
    pub fn new(
        uart: UartType,
        uart_dir: Option<UartDirType>,
        weight_sensors: WeightSensors,
    ) -> Self {
        Self { uart, uart_dir, weight_sensors, weight_zero_values: None }
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

    async fn wait_for_dtu_available(&mut self) -> Result<(), AtomiError> {
        loop {
            let t = self
                .forward(AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::WaitIdle)))
                .await;
            match t {
                Ok(AtomiProto::AtomiError(AtomiError::DtuUnavailable)) => {
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

    async fn wait_for_stepper_available(&mut self) -> Result<(), AtomiError> {
        loop {
            let t = self
                .forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::WaitIdle)))
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

    async fn weight_sensor_init(&mut self) -> Result<(), AtomiError> {
        let (w1, w2, w3, w4) = self.weight_sensors.get_all_weights().await;
        debug!("raw weight values: {}, {}, {}, {}", w1, w2, w3, w4);
        self.weight_zero_values = Some([w1, w2, w3, w4]);
        Ok(())
    }

    async fn calc_weight(&mut self) -> Result<i32, AtomiError> {
        let (w1, w2, w3, w4) = self.weight_sensors.get_all_weights().await;
        debug!("raw weight values: {}, {}, {}, {}", w1, w2, w3, w4);
        if let Some([zw1, zw2, zw3, zw4]) = self.weight_zero_values {
            let w_sum = w1 as i32 - zw1 as i32 + w2 as i32 - zw2 as i32 + w3 as i32 - zw3 as i32
                + w4 as i32
                - zw4 as i32;
            Ok(w_sum)
        } else {
            Err(AtomiError::McWeightSensorNotInited)
        }
    }

    async fn wait_for_linear_bull_available(&mut self) -> Result<(), AtomiError> {
        loop {
            let t = self
                .forward(AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::WaitIdle)))
                .await;

            // Check pressure.
            // TODO: To Zephyr, HPD stop not working
            // DONE: HPD stop issue fixed

            if let Ok(weight) = self.calc_weight().await {
                if weight > 0 && weight as u32 > LINEAR_BULL_MAX_PRESSURE {
                    info!("hpd stop due to exceed max pressure: {}", weight);
                    return self.hpd_stop().await;
                }
            }

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

    async fn mmd_stepper_home(&mut self) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::Home)))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn dtu_home(&mut self) -> Result<(), AtomiError> {
        let res =
            self.forward(AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::Home))).await?;
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

    #[allow(dead_code)]
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

    async fn hpd_move_to(&mut self, position: i32) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::MoveTo {
                position,
            })))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn dtu_move_to(&mut self, position: i32, speed: u32) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::MoveTo {
                position,
                speed,
            })))
            .await?;
        expect_result(res, AtomiProto::Unknown)
    }

    async fn mmd_move_to(&mut self, position: i32, speed: u32) -> Result<(), AtomiError> {
        let res = self
            .forward(AtomiProto::Mmd(MmdCommand::MmdLinearStepper(StepperCommand::MoveTo {
                position,
                speed,
            })))
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

    pub async fn vibratory_on(&mut self) -> Result<(), AtomiError> {
        //-1000 ~ -1的转速范围会启动振动马达，1-1000的转速范围马达会被掐断供电
        //-1000转速最低，-1转速最高
        self.mmd_dispenser(1, -1).await // set vibratory motor to maximum speed
    }

    pub async fn vibratory_off(&mut self) -> Result<(), AtomiError> {
        self.mmd_dispenser(1, 980).await // set vibratory motor to slow speed
    }

    pub async fn system_init(&mut self) -> Result<(), AtomiError> {
        // Init the system
        self.mmd_stepper_home().await?;
        self.hpd_linear_bull_home().await?;
        self.dtu_home().await?;
        self.vibratory_off().await?; // set vibratory motor to slow state

        self.mmd_pr_off().await?; // self.pr_set(off)
        self.mmd_pp_off().await?; // self.pp_set(off)
        self.mmd_dispenser_off(0).await?; // self.dispenser(0, off)
        self.mmd_belt_off().await?; // self.belt_set(off)

        self.wait_for_stepper_available().await?;
        self.wait_for_linear_bull_available().await?;
        self.wait_for_dtu_available().await?;

        self.weight_sensor_init().await?;
        Ok(())
    }

    pub async fn squeeze_ketchup(&mut self) -> Result<(), AtomiError> {
        // 挤番茄酱
        // init
        self.mmd_pr(200).await?;
        self.mmd_pp(-300).await?;

        self.mmd_move_to(200, 500).await?;
        self.mmd_pr(218).await?;
        self.wait_for_stepper_available().await?;
        Delay::new(6329.millis()).await;

        self.mmd_move_to(303, 500).await?;
        self.mmd_pr(277).await?;
        self.wait_for_stepper_available().await?;
        Delay::new(4973.millis()).await;

        self.mmd_move_to(406, 500).await?;
        self.mmd_pr(381).await?;
        self.wait_for_stepper_available().await?;
        Delay::new(3617.millis()).await;

        // 加速度
        self.mmd_pr(500).await?;
        Delay::new(300.millis()).await;

        self.mmd_move_to(510, 500).await?;
        self.mmd_pr(611).await?;
        self.wait_for_stepper_available().await?;
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
        // self.wait_for_stepper_available().await?;

        // 传送带伸出去
        self.mmd_move_to(510, 400).await?;

        // 转盘启动
        self.mmd_pr((459_f32 * spd_amp1) as i32).await?;
        // 等待传送带运行就位
        self.wait_for_stepper_available().await?;

        // 加速度
        self.mmd_pr(620).await?;
        Delay::new(200.millis()).await;

        // 转盘达到正式速度
        self.mmd_pr((759_f32 * spd_amp1) as i32).await?;

        // init
        // 初始化，料斗启动，正转
        self.vibratory_on().await?;
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
        //self.wait_for_stepper_available().await?;
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
        self.wait_for_stepper_available().await?;
        // 重新启动传送带
        self.mmd_belt(BELT_ON_SPEED).await?;
        // 重新启动料斗，正转
        self.mmd_dispenser(0, DISPENSER_ON_SPEED).await?;
        // // 抵消反转造成的非连续性
        // Delay::new(((delay_revert) as u64).millis()).await;
        //        Delay::new(((delay_revert) as u64).millis()).await;

        // 加速度
        self.mmd_pr(200).await?;
        Delay::new(200.millis()).await;

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
        self.wait_for_stepper_available().await?;
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

        // 加速度
        self.mmd_pr(500).await?;
        Delay::new(200.millis()).await;

        // 转盘提速
        self.mmd_pr(700).await?;
        // 等转过去一点再启动
        Delay::new(600_u64.millis()).await;

        // 加速度
        self.mmd_pr(500).await?;
        Delay::new(200.millis()).await;

        // 转盘恢复之前的速度
        self.mmd_pr((292_f32 * spd_amp3) as i32).await?;
        // 恢复传送带速度
        self.mmd_belt(BELT_ON_SPEED).await?;

        // 等待传送带上剩余起司全部送出
        self.mmd_dispenser_off(0).await?;
        self.vibratory_off().await?;
        Delay::new(1800u64.millis()).await;
        Ok(())
    }

    pub async fn make_one_pizza(&mut self) -> Result<(), AtomiError> {
        let hpd_end_pos = 56800; // 压饼的终点位置  56000
        let hpd_dtu_pos = 49500; // 抬高到能让不沾铲插入的位置
        let hpd_pre_end_pos = 55000; // 压饼之后抬高但并不会把饼带太高的位置，也即刚好等于饼松弛状态下的厚度
        let hpd_dock_pos = 22000; // 平时压饼悬空的位置。

        let dtu_insert_pos = 12000; // 不沾铲刚开始插入的位置
        let dtu_end_pos = 45000; // 不沾铲伸出的极限位置

        // 压面团
        self.hpd_move_to(hpd_end_pos).await?;
        self.wait_for_linear_bull_available().await?;
        Delay::new(3.secs()).await;

        // 抬高一点
        self.hpd_move_to(hpd_dtu_pos).await?;
        self.wait_for_linear_bull_available().await?;

        // 铲入一部分
        self.dtu_move_to(dtu_insert_pos, 5000).await?;
        self.wait_for_dtu_available().await?;

        // 压下去一点
        self.hpd_move_to(hpd_pre_end_pos).await?;
        self.wait_for_linear_bull_available().await?;

        // 继续铲下去
        self.dtu_move_to(dtu_end_pos, 5000).await?;
        self.wait_for_dtu_available().await?;

        // 抽回
        self.dtu_move_to(0, 7000).await?;
        self.wait_for_dtu_available().await?;

        self.hpd_move_to(hpd_dock_pos).await?;
        self.wait_for_linear_bull_available().await?;

        // 涂番茄酱
        self.squeeze_ketchup().await?;
        // 撒起司
        self.sprinkle_cheese().await?;

        // 结束
        self.mmd_move_to(0, 500).await?;
        // self.hpd_move_to(0).await?;
        self.wait_for_stepper_available().await?;
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
            Some(AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::WeightSensorInit))) => {
                error_or_done(executor.weight_sensor_init().await, mq_out);
            }
            Some(AtomiProto::Mc(McCommand::SystemRun(McSystemExecutorCmd::GetWeight))) => {
                match executor.calc_weight().await {
                    Ok(weight) => mq_out.enqueue(McSystemExecutorResponse::Weight(weight)),
                    Err(err) => mq_out.enqueue(McSystemExecutorResponse::Error(err)),
                }
            }
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
