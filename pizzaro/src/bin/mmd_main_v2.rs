#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use can2040::CanFrame;
use core::sync::atomic::Ordering;

use cortex_m::asm::delay;
use defmt::{debug, info, Debug2Format};
use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU64;
use rp2040_hal::{clocks::init_clocks_and_plls, entry, pac, sio::Sio, watchdog::Watchdog, Timer};
use rp_pico::XOSC_CRYSTAL_FREQ;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{
    AtomiProto, MmdCommand, PeristalticPumpCommand, StepperCommand, StepperResponse,
};
use generic::mmd_status::MmdStatus;
use pizzaro::bsp::config::{
    MMD_BRUSHLESS_MOTOR_PWM_TOP, MMD_PERISTALTIC_PUMP_PWM_TOP, REVERT_MMD_STEPPER42_0_DIRECTION,
    REVERT_MMD_STEPPER42_1_DIRECTION, REVERT_MMD_STEPPER57_DIRECTION,
};
use pizzaro::bsp::{
    MmdBrushMotorChannel, MmdBrushlessMotor0Channel, MmdBrushlessMotor1Channel,
    MmdMotor42Step1Channel, MmdMotor57StepChannel,
};
use pizzaro::common::brush_motor_patch::BrushMotorPatched;
use pizzaro::common::brushless_motor::BrushlessMotor;
use pizzaro::common::can_messenger::{get_can_messenger, init_can_messenger, parse_frame};
use pizzaro::common::consts::{
    BELT_OFF_SPEED, CANBUS_FREQUENCY, DISPENSER_OFF_SPEED, MMD_CAN_ID, PP_OFF_SPEED, PR_OFF_SPEED,
};
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, Delay, DelayCreator};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::pwm_stepper::PwmStepper;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::stepper_driver::StepperDriver;
use pizzaro::mmd::brush_motor_processor::MmdPeristalicPumpProcessor;
use pizzaro::mmd::brushless_motor_processor::DispenserMotorProcessor;
use pizzaro::mmd::rotation_stepper_processor::RotationStepperProcessor;
use pizzaro::mmd::stepper::Stepper;
use pizzaro::mmd::stepper_processor::{
    process_mmd_stepper_message, stepper_input_mq, stepper_output_mq, LinearStepperProcessor,
};
use pizzaro::mmd::GLOBAL_STEPPER_STOP;
use pizzaro::{common::async_initialization, mmd_sys_rx, mmd_sys_tx};
use pizzaro::{
    mmd_bl1_ctl_channel, mmd_bl1_ctl_pwm_slice, mmd_bl2_ctl_channel, mmd_bl2_ctl_pwm_slice,
    mmd_br0_pwm_slice, mmd_br_channel_a, mmd_br_nEN, mmd_br_pwm_a, mmd_br_pwm_b, mmd_dir_bl0,
    mmd_dir_bl1, mmd_limit0, mmd_limit1, mmd_motor42_pwm_slice1, mmd_motor42_step1_channel,
    mmd_spd_ctrl_bl0, mmd_spd_ctrl_bl1, mmd_stepper42_dir0, mmd_stepper42_dir1, mmd_stepper42_nEN0,
    mmd_stepper42_nEN1, mmd_stepper42_step0, mmd_stepper42_step1, mmd_stepper57_dir,
    mmd_stepper57_nEN, mmd_stepper57_pwm_slice, mmd_stepper57_step, mmd_stepper57_step_channel,
    mmd_tmc_uart_tx,
};

// TODO(zephyr): 把下面结构改成Once<>结构.
static mut DISPENSER_MOTOR_PROCESSOR: Option<DispenserMotorProcessor> = None;
static mut PERISTALIC_PUMP_PROCESSOR: Option<MmdPeristalicPumpProcessor> = None;
static mut ROTATION_STEPPER_PROCESSOR: Option<RotationStepperProcessor> = None;

static mut MESSAGE_MMD_QUEUE_ONCE: Once<MessageQueueWrapper<CanFrame>> = Once::new();
fn get_mmd_mq() -> &'static mut MessageQueueWrapper<CanFrame> {
    unsafe { MESSAGE_MMD_QUEUE_ONCE.get_mut() }
}

#[entry]
fn main() -> ! {
    async_initialization();
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    init_global_timer(Box::new(Rp2040Timer::new(timer)));

    let pins =
        rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    {
        // init CAN bus here.
        init_can_messenger(
            MMD_CAN_ID,
            &mut core,
            CANBUS_FREQUENCY,
            mmd_sys_rx!(pins).id().num as u32,
            mmd_sys_tx!(pins).id().num as u32,
        );
        get_can_messenger().set_default_queue(get_mmd_mq());
        spawn_task(get_can_messenger().receive_task());
    }

    let pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    {
        let mut pwm0 = mmd_bl1_ctl_pwm_slice!(pwm_slices);
        pwm0.set_ph_correct();
        pwm0.set_top(MMD_BRUSHLESS_MOTOR_PWM_TOP);
        pwm0.enable();
        mmd_bl1_ctl_channel!(pwm0).output_to(mmd_spd_ctrl_bl0!(pins));

        let mut pwm1 = mmd_bl2_ctl_pwm_slice!(pwm_slices);
        pwm1.set_ph_correct();
        pwm1.set_top(MMD_BRUSHLESS_MOTOR_PWM_TOP);
        pwm1.enable();
        mmd_bl2_ctl_channel!(pwm1).output_to(mmd_spd_ctrl_bl1!(pins));

        let dispenser0_motor = BrushlessMotor::new(
            mmd_dir_bl0!(pins).into_push_pull_output().into_dyn_pin(),
            pwm0,
            MmdBrushlessMotor0Channel,
            (0.03, 0.45, 0.55, 0.97),
            false,
            MMD_BRUSHLESS_MOTOR_PWM_TOP,
        );
        let dispenser1_motor = BrushlessMotor::new(
            mmd_dir_bl1!(pins).into_push_pull_output().into_dyn_pin(),
            pwm1,
            MmdBrushlessMotor1Channel,
            (0.03, 0.45, 0.55, 0.97),
            false,
            MMD_BRUSHLESS_MOTOR_PWM_TOP,
        );
        let dispenser_motor_processor =
            DispenserMotorProcessor::new(dispenser0_motor, dispenser1_motor);

        unsafe {
            DISPENSER_MOTOR_PROCESSOR = Some(dispenser_motor_processor);
        }
    }

    {
        let mut pwm = mmd_br0_pwm_slice!(pwm_slices);
        pwm.set_ph_correct();
        pwm.set_top(MMD_PERISTALTIC_PUMP_PWM_TOP);

        pwm.enable();
        mmd_br_channel_a!(pwm).output_to(mmd_br_pwm_a!(pins));
        //mmd_br_channel_b!(pwm).output_to(mmd_br_pwm_b!(pins));
        //mmd_br_channel_b!(pwm).set_inverted();

        let peristaltic_pump_motor = BrushMotorPatched::new(
            mmd_br_nEN!(pins).into_push_pull_output().into_dyn_pin(),
            mmd_br_pwm_b!(pins).into_push_pull_output().into_dyn_pin(), // pwmb as dir pin
            pwm,
            MmdBrushMotorChannel,
            (0.03, 0.45, 0.55, 0.97),
            false,
            MMD_PERISTALTIC_PUMP_PWM_TOP,
        );
        let peristalic_pump_processor = MmdPeristalicPumpProcessor::new(peristaltic_pump_motor);
        unsafe {
            PERISTALIC_PUMP_PROCESSOR = Some(peristalic_pump_processor);
        }
    }

    // init tmc pin
    let mut tmc_uart = mmd_tmc_uart_tx!(pins).into_pull_down_disabled().into_push_pull_output();
    tmc_uart.set_low().unwrap();

    debug!("hello world");
    {
        spawn_task(mmd_process_messages());
    }
    {
        // 使用第一个通道连接驱动伸缩的电机
        let enable_pin = mmd_stepper42_nEN0!(pins).into_push_pull_output();
        let dir_pin = mmd_stepper42_dir0!(pins).into_push_pull_output();

        let step_pin = mmd_stepper42_step0!(pins).into_push_pull_output();
        let left_limit_pin = mmd_limit0!(pins).into_pull_down_input();
        let right_limit_pin = mmd_limit1!(pins).into_pull_down_input();
        let delay_creator = DelayCreator::new();
        let processor = LinearStepperProcessor::new(Stepper::new(
            StepperDriver::new(
                enable_pin,
                dir_pin,
                step_pin,
                delay_creator,
                REVERT_MMD_STEPPER42_0_DIRECTION,
            ),
            left_limit_pin,
            right_limit_pin,
        ));
        spawn_task(process_mmd_stepper_message(processor));
    }
    {
        // 使用第二个通道连接驱动传送带旋转的电机
        let enable_pin_42 = mmd_stepper42_nEN1!(pins);
        let dir_pin_42 = mmd_stepper42_dir1!(pins).into_push_pull_output().into_dyn_pin();
        //let step_pin_42 = mmd_stepper42_step1!(pins).into_push_pull_output();

        let mut pwm_42 = mmd_motor42_pwm_slice1!(pwm_slices);
        mmd_motor42_step1_channel!(pwm_42).output_to(mmd_stepper42_step1!(pins));

        // init 57
        let enable_pin_57 = Some(mmd_stepper57_nEN!(pins).into_push_pull_output().into_dyn_pin());
        let dir_pin_57 = mmd_stepper57_dir!(pins).into_push_pull_output().into_dyn_pin();
        //let step_pin_57 = mmd_stepper57_step!(pins).into_push_pull_output();

        let mut pwm_57 = mmd_stepper57_pwm_slice!(pwm_slices);
        mmd_stepper57_step_channel!(pwm_57).output_to(mmd_stepper57_step!(pins));

        let processor = RotationStepperProcessor::new(
            PwmStepper::new(
                enable_pin_42,
                dir_pin_42,
                MmdMotor42Step1Channel,
                pwm_42,
                200, // 无细分，一圈200脉冲
                REVERT_MMD_STEPPER42_1_DIRECTION,
            ),
            PwmStepper::new(
                enable_pin_57,
                dir_pin_57,
                MmdMotor57StepChannel,
                pwm_57,
                200, // 无细分，一圈200脉冲
                REVERT_MMD_STEPPER57_DIRECTION,
            ),
        );
        unsafe {
            ROTATION_STEPPER_PROCESSOR = Some(processor);
        }
    }

    // spawn_task(dump_executor_status());
    start_global_executor();
    GLOBAL_STEPPER_STOP.store(false, Ordering::Relaxed);

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

// MMD main future
async fn mmd_process_messages() {
    debug!("[MMD] mmd_process_messages 0");
    let mut mmd_linear_stepper_available = true;
    let dispenser_motor_processor = unsafe { DISPENSER_MOTOR_PROCESSOR.as_mut().unwrap() };
    let peristalic_pump_processor = unsafe { PERISTALIC_PUMP_PROCESSOR.as_mut().unwrap() };
    let rotation_stepper_processor = unsafe { ROTATION_STEPPER_PROCESSOR.as_mut().unwrap() };
    loop {
        if let Some(frame) = get_mmd_mq().dequeue() {
            info!("[MMD] mmd_process_messages() 1.1 | dequeued message: {}", Debug2Format(&frame));
            let parse_res = parse_frame::<MmdCommand>(frame);
            if let Err(err) = parse_res {
                info!(
                    "[MMD] mmd_process_messages(): errors in parsing frame, {}",
                    Debug2Format(&err)
                );
                continue;
            }
            let (resp_id, mmd_cmd) = parse_res.unwrap();
            let process_res = match mmd_cmd {
                MmdCommand::MmdPing => get_can_messenger().send_raw(resp_id, MmdCommand::MmdPong),

                MmdCommand::MmdStop => {
                    if !mmd_linear_stepper_available {
                        // If linear_stepper is running, stop it directly.
                        GLOBAL_STEPPER_STOP.store(true, Ordering::Relaxed);
                    }

                    // Stop other components.
                    peristalic_pump_processor.set_speed(PP_OFF_SPEED);
                    dispenser_motor_processor.set_dispenser0_speed(DISPENSER_OFF_SPEED);
                    dispenser_motor_processor.set_dispenser1_speed(DISPENSER_OFF_SPEED);
                    rotation_stepper_processor.set_conveyor_speed(BELT_OFF_SPEED);
                    rotation_stepper_processor.set_presser_speed(PR_OFF_SPEED);

                    if !mmd_linear_stepper_available {
                        // wait till the linear_stepper is stopped.
                        loop {
                            if let Some(linear_stepper_resp) = stepper_output_mq().dequeue() {
                                info!("linear_stepper_resp: {}", linear_stepper_resp);
                                assert_eq!(
                                    linear_stepper_resp,
                                    StepperResponse::Error(AtomiError::MmdStopped)
                                );
                                break;
                            }
                        }
                        GLOBAL_STEPPER_STOP.store(false, Ordering::Relaxed);
                        mmd_linear_stepper_available = true;
                    }
                    get_can_messenger().send_raw(resp_id, MmdCommand::MmdAck)
                }

                MmdCommand::MmdLinearStepper(StepperCommand::WaitIdle) => {
                    // // 处理wait idle 不能受 mmd_linear_stepper_available限制，先用这个办法workaround掉
                    // let res = uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdAck));
                    // linear_stepper_input_mq().enqueue(LinearStepperCommand::WaitIdle);
                    // res

                    if mmd_linear_stepper_available {
                        get_can_messenger().send_raw(resp_id, MmdCommand::MmdAck)
                    } else {
                        let _ = get_can_messenger()
                            .send_raw(resp_id, MmdCommand::MmdError(AtomiError::HpdUnavailable));
                        Err(AtomiError::MmdUnavailable(MmdStatus::Unavailable))
                    }
                }

                MmdCommand::MmdLinearStepper(cmd) => {
                    if mmd_linear_stepper_available {
                        let res = get_can_messenger().send_raw(resp_id, MmdCommand::MmdAck);
                        stepper_input_mq().enqueue(cmd);
                        mmd_linear_stepper_available = false;
                        res
                    } else {
                        let _ = get_can_messenger()
                            .send_raw(resp_id, MmdCommand::MmdError(AtomiError::HpdUnavailable));
                        Err(AtomiError::MmdUnavailable(MmdStatus::Unavailable))
                    }
                }

                MmdCommand::MmdRotationStepper(cmd) => {
                    match rotation_stepper_processor.process_rotation_stepper_request(cmd) {
                        Ok(AtomiProto::Mmd(msg)) => get_can_messenger().send_raw(resp_id, msg),
                        Err(err) => {
                            get_can_messenger().send_raw(resp_id, MmdCommand::MmdError(err))
                        }
                        _ => Ok(()),
                    }
                }

                MmdCommand::MmdDisperser(cmd) => {
                    dispenser_motor_processor.process(cmd).unwrap();
                    // info!("send ack 1");
                    get_can_messenger().send_raw(resp_id, MmdCommand::MmdAck)
                }

                MmdCommand::MmdPeristalticPump(cmd) => {
                    let PeristalticPumpCommand::SetRotation { speed } = cmd;
                    peristalic_pump_processor.set_speed(speed);
                    get_can_messenger().send_raw(resp_id, MmdCommand::MmdAck)
                }

                _ => Err(AtomiError::IgnoredMsg), // Ignore unrelated commands
            };

            if let Err(err) = process_res {
                info!("[MMD] message processing error: {}", err);
                continue;
            }
        }

        if let Some(linear_stepper_resp) = stepper_output_mq().dequeue() {
            info!("[MMD] get response from linear stepper: {}", linear_stepper_resp);
            mmd_linear_stepper_available = true;
        }

        // 延迟一段时间
        Delay::new(1.millis()).await;
    }
}
