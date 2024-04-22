#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec;
use core::sync::atomic::Ordering;
use can2040::{CanError, CanFrame};

use cortex_m::asm::delay;
use cortex_m::peripheral::NVIC;
use defmt::{debug, error, info, Debug2Format};
use embedded_can::blocking::Can;
use embedded_hal::digital::v2::OutputPin;
use fugit::{ExtU64, RateExtU32};
use rp2040_hal::gpio::FunctionUart;
use rp2040_hal::uart::{DataBits, StopBits, UartConfig};
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry, pac,
    pac::interrupt,
    sio::Sio,
    uart::UartPeripheral,
    watchdog::Watchdog,
    Timer,
};
use rp_pico::XOSC_CRYSTAL_FREQ;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::MmdCommand::MmdBusy;
use generic::atomi_proto::{wrap_result_into_proto, AtomiProto, LinearStepperCommand, LinearStepperResponse, MmdCommand, PeristalticPumpCommand, AtomiErrorWithCanId};
use generic::mmd_status::MmdStatus;
use pizzaro::bsp::config::{
    MMD_BRUSHLESS_MOTOR_PWM_TOP, MMD_PERISTALTIC_PUMP_PWM_TOP, REVERT_MMD_STEPPER42_0_DIRECTION,
    REVERT_MMD_STEPPER42_1_DIRECTION, REVERT_MMD_STEPPER57_DIRECTION,
};
use pizzaro::bsp::{
    mmd_uart_irq, MmdBrushMotorChannel, MmdBrushlessMotor0Channel, MmdBrushlessMotor1Channel,
    MmdMotor42Step1Channel, MmdMotor57StepChannel, MmdUartDirPinType, MmdUartType,
};
use pizzaro::common::brush_motor_patch::BrushMotorPatched;
use pizzaro::common::brushless_motor::BrushlessMotor;
use pizzaro::common::consts::{BELT_OFF_SPEED, CANBUS_FREQUENCY, DISPENSER_OFF_SPEED, MC_CAN_ID, MMD_CAN_ID, PP_OFF_SPEED, PR_OFF_SPEED, UART_EXPECTED_RESPONSE_LENGTH};
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_status::{get_status, FutureStatus, FutureType};
use pizzaro::common::global_timer::{init_global_timer, now, Delay, DelayCreator};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::pwm_stepper::PwmStepper;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::uart_comm::UartComm;
use pizzaro::mmd::brush_motor_processor::MmdPeristalicPumpProcessor;
use pizzaro::mmd::brushless_motor_processor::DispenserMotorProcessor;
use pizzaro::mmd::linear_stepper::LinearStepper;
use pizzaro::mmd::linear_stepper_processor::{
    linear_stepper_input_mq, linear_stepper_output_mq, process_mmd_linear_stepper_message,
    LinearStepperProcessor,
};
use pizzaro::mmd::rotation_stepper_processor::RotationStepperProcessor;
use pizzaro::mmd::stepper::Stepper;
use pizzaro::mmd::GLOBAL_LINEAR_STEPPER_STOP;
use pizzaro::{common::async_initialization, hpd_sys_rx, hpd_sys_tx, mmd_sys_rx, mmd_sys_tx};
use pizzaro::{
    mmd_485_dir, mmd_bl1_ctl_channel, mmd_bl1_ctl_pwm_slice, mmd_bl2_ctl_channel,
    mmd_bl2_ctl_pwm_slice, mmd_br0_pwm_slice, mmd_br_channel_a, mmd_br_nEN, mmd_br_pwm_a,
    mmd_br_pwm_b, mmd_dir_bl0, mmd_dir_bl1, mmd_limit0, mmd_limit1, mmd_motor42_pwm_slice1,
    mmd_motor42_step1_channel, mmd_spd_ctrl_bl0, mmd_spd_ctrl_bl1, mmd_stepper42_dir0,
    mmd_stepper42_dir1, mmd_stepper42_nEN0, mmd_stepper42_nEN1, mmd_stepper42_step0,
    mmd_stepper42_step1, mmd_stepper57_dir, mmd_stepper57_nEN, mmd_stepper57_pwm_slice,
    mmd_stepper57_step, mmd_stepper57_step_channel, mmd_tmc_uart_tx, mmd_uart,
};
use pizzaro::common::can::{enqueue_can_frame_if_possible, get_can, init_can_bus, send_can_message};

// TODO(zephyr): 把下面结构改成Once<>结构.
static mut DISPENSER_MOTOR_PROCESSOR: Option<DispenserMotorProcessor> = None;
static mut PERISTALIC_PUMP_PROCESSOR: Option<MmdPeristalicPumpProcessor> = None;
static mut ROTATION_STEPPER_PROCESSOR: Option<RotationStepperProcessor> = None;

static mut MESSAGE_MMD_QUEUE_ONCE: Once<MessageQueueWrapper<MmdCommand>> = Once::new();
fn get_mmd_mq() -> &'static mut MessageQueueWrapper<MmdCommand> {
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
        init_can_bus(
            &mut core,
            CANBUS_FREQUENCY,
            mmd_sys_rx!(pins).id().num as u32,
            mmd_sys_tx!(pins).id().num as u32,
        );
        spawn_task(process_can_mmd_message());
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
        let processor = LinearStepperProcessor::new(LinearStepper::new(
            Stepper::new(
                enable_pin,
                dir_pin,
                step_pin,
                delay_creator,
                REVERT_MMD_STEPPER42_0_DIRECTION,
            ),
            left_limit_pin,
            right_limit_pin,
        ));
        spawn_task(process_mmd_linear_stepper_message(processor));
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

    start_global_executor();
    GLOBAL_LINEAR_STEPPER_STOP.store(false, Ordering::Relaxed);

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
        if let Some(message) = get_mmd_mq().dequeue() {
            info!("[MMD] process_messages() 1.1 | dequeued message: {}", message);

            // 处理消息
            let res = match message {
                AtomiProto::Mmd(MmdCommand::MmdPing) => {
                    send_can_message(MC_CAN_ID, MmdCommand::MmdPong)
                }

                AtomiProto::Mmd(MmdCommand::MmdStop) => {
                    if !mmd_linear_stepper_available {
                        // If linear_stepper is running, stop it directly.
                        GLOBAL_LINEAR_STEPPER_STOP.store(true, Ordering::Relaxed);
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
                            if let Some(linear_stepper_resp) = linear_stepper_output_mq().dequeue()
                            {
                                info!("linear_stepper_resp: {}", linear_stepper_resp);
                                assert_eq!(
                                    linear_stepper_resp,
                                    LinearStepperResponse::Error(AtomiError::MmdStopped)
                                );
                                break;
                            }
                        }
                        GLOBAL_LINEAR_STEPPER_STOP.store(false, Ordering::Relaxed);
                        mmd_linear_stepper_available = true;
                    }
                    send_can_message(MC_CAN_ID, MmdCommand::MmdAck)
                }

                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::WaitIdle)) => {
                    // // 处理wait idle 不能受 mmd_linear_stepper_available限制，先用这个办法workaround掉
                    // let res = uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdAck));
                    // linear_stepper_input_mq().enqueue(LinearStepperCommand::WaitIdle);
                    // res

                    if mmd_linear_stepper_available {
                        send_can_message(MC_CAN_ID, MmdCommand::MmdAck)
                    } else {
                        let _ = send_can_message(MC_CAN_ID, AtomiErrorWithCanId::new(
                            MMD_CAN_ID, AtomiError::MmdUnavailable(MmdStatus::Unavailable)));
                        Err(AtomiError::MmdUnavailable(MmdStatus::Unavailable))
                    }
                }

                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(cmd)) => {
                    if mmd_linear_stepper_available {
                        let res = send_can_message(MC_CAN_ID, MmdCommand::MmdAck);
                        linear_stepper_input_mq().enqueue(cmd);
                        mmd_linear_stepper_available = false;
                        res
                    } else {
                        let _ = send_can_message(MC_CAN_ID, AtomiErrorWithCanId::new(
                            MMD_CAN_ID, AtomiError::MmdUnavailable(MmdStatus::Unavailable)));
                        Err(AtomiError::MmdUnavailable(MmdStatus::Unavailable))
                    }
                }

                AtomiProto::Mmd(MmdCommand::MmdRotationStepper(cmd)) => {
                    match rotation_stepper_processor.process_rotation_stepper_request(cmd) {
                        Ok(AtomiProto::Mmd(msg)) => {
                            send_can_message(MC_CAN_ID, msg)
                        }
                        Err(err) => {
                            send_can_message(MC_CAN_ID, AtomiErrorWithCanId::new(MMD_CAN_ID, err))
                        }
                        _ => Ok(())
                    }
                }

                AtomiProto::Mmd(MmdCommand::MmdDisperser(cmd)) => {
                    dispenser_motor_processor.process(cmd).unwrap();
                    // info!("send ack 1");
                    send_can_message(MC_CAN_ID, MmdCommand::MmdAck)
                }

                AtomiProto::Mmd(MmdCommand::MmdPeristalticPump(cmd)) => {
                    let PeristalticPumpCommand::SetRotation { speed } = cmd;
                    peristalic_pump_processor.set_speed(speed);
                    send_can_message(MC_CAN_ID, MmdCommand::MmdAck)
                }

                _ => Err(AtomiError::IgnoredMsg), // Ignore unrelated commands
            };

            if let Err(err) = res {
                info!("[MMD] message processing error: {}", err);
                continue;
            }
        }

        if let Some(linear_stepper_resp) = linear_stepper_output_mq().dequeue() {
            info!("[MMD] get response from linear stepper: {}", linear_stepper_resp);
            mmd_linear_stepper_available = true;
        }

        // 延迟一段时间
        Delay::new(1.millis()).await;
    }
}

async fn process_can_mmd_message() {
    loop {
        Delay::new(100.micros()).await;

        match get_can().receive() {
            Ok(f) => {
                enqueue_can_frame_if_possible(MMD_CAN_ID, f, get_mmd_mq())
            }
            Err(nb::Error::Other(err)) => {
                error!("Canbus error: {}", err);
            }
            Err(_) => (),
        }
    }
}