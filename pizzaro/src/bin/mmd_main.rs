#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec;

use cortex_m::asm::delay;
use cortex_m::peripheral::NVIC;
use defmt::{debug, error, info, Debug2Format};
use embedded_hal::digital::v2::OutputPin;
use fugit::{ExtU64, RateExtU32};
// use pizzaro::common::brush_motor::{BrushMotor, MMD_PWM_TOP};
use pizzaro::common::brush_motor_patch::{BrushMotorPatch, MMD_PWM_TOP};

use pizzaro::common::brushless_motor::BrushlessMotor;
use pizzaro::common::pwm_stepper::PwmStepper;
use pizzaro::mmd::brush_motor_processor::BrushMotorProcessor;
use pizzaro::mmd::brushless_motor_processor::BrushlessMotorProcessor;
use pizzaro::mmd::rotation_stepper_processor::{
    process_mmd_rotation_stepper_message, rotation_stepper_input_mq, rotation_stepper_output_mq,
    RotationStepperProcessor,
};

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

use generic::atomi_error::AtomiError;
use generic::atomi_proto::MmdCommand::MmdBusy;
use generic::atomi_proto::{AtomiProto, LinearStepperCommand, MmdCommand};
use generic::mmd_status::MmdStatus;
use pizzaro::bsp::{
    mmd_uart_irq, MmdBrushMotorChannel, MmdBrushlessMotor0Channel, MmdBrushlessMotor1Channel,
    MmdMotor42Step1Channel, MmdMotor57StepChannel, MmdUartDirPinType, MmdUartType,
    MMD_STEPPER42_0_REVERT_DIR, MMD_STEPPER42_1_REVERT_DIR, MMD_STEPPER57_REVERT_DIR,
};
use pizzaro::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_status::{get_status, FutureStatus, FutureType};
use pizzaro::common::global_timer::{init_global_timer, now, Delay, DelayCreator};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::uart_comm::UartComm;
use pizzaro::mmd::linear_stepper::LinearStepper;
use pizzaro::mmd::linear_stepper_processor::{
    linear_stepper_input_mq, linear_stepper_output_mq, process_mmd_linear_stepper_message,
    LinearStepperProcessor,
};
use pizzaro::mmd::stepper::Stepper;
use pizzaro::{common::async_initialization, mmd_sys_rx, mmd_sys_tx};
use pizzaro::{
    mmd_485_dir, mmd_bl1_ctl_channel, mmd_bl1_ctl_pwm_slice, mmd_bl2_ctl_channel,
    mmd_bl2_ctl_pwm_slice, mmd_br0_pwm_slice, mmd_br_channel_a, mmd_br_nEN, mmd_br_pwm_a,
    mmd_br_pwm_b, mmd_dir_bl0, mmd_dir_bl1, mmd_limit0, mmd_limit1, mmd_motor42_pwm_slice1,
    mmd_motor42_step1_channel, mmd_spd_ctrl_bl0, mmd_spd_ctrl_bl1, mmd_stepper42_dir0,
    mmd_stepper42_dir1, mmd_stepper42_nEN0, mmd_stepper42_nEN1, mmd_stepper42_step0,
    mmd_stepper42_step1, mmd_stepper57_dir, mmd_stepper57_nEN, mmd_stepper57_pwm_slice,
    mmd_stepper57_step, mmd_stepper57_step_channel, mmd_tmc_uart_tx, mmd_uart,
};
use rp_pico::XOSC_CRYSTAL_FREQ;

static mut UART: Option<(MmdUartType, Option<MmdUartDirPinType>)> = None;
static mut BRUSHLESS_MOTOR_PROCESSOR: Option<BrushlessMotorProcessor> = None;
static mut BRUSH_MOTOR_PROCESSOR: Option<BrushMotorProcessor> = None;

static mut MESSAGE_QUEUE_ONCE: Once<MessageQueueWrapper<AtomiProto>> = Once::new();
fn get_mq() -> &'static mut MessageQueueWrapper<AtomiProto> {
    unsafe { MESSAGE_QUEUE_ONCE.get_mut() }
}

#[entry]
fn main() -> ! {
    async_initialization();
    let mut pac = pac::Peripherals::take().unwrap();
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

    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let uart_pins = (
        mmd_sys_tx!(pins).into_function::<FunctionUart>(), // TX, not used in this program
        mmd_sys_rx!(pins).into_function::<FunctionUart>(), // RX
    );
    let uart_dir = mmd_485_dir!(pins).reconfigure();

    let mut uart = UartPeripheral::new(mmd_uart!(pac), uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.enable_rx_interrupt();
    unsafe {
        UART = Some((uart, Some(uart_dir)));
        NVIC::unmask(mmd_uart_irq());
    }

    let pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    {
        let mut pwm0 = mmd_bl1_ctl_pwm_slice!(pwm_slices);
        pwm0.set_ph_correct();
        pwm0.set_top(MMD_PWM_TOP);
        pwm0.enable();
        mmd_bl1_ctl_channel!(pwm0).output_to(mmd_spd_ctrl_bl0!(pins));

        let mut pwm1 = mmd_bl2_ctl_pwm_slice!(pwm_slices);
        pwm1.set_ph_correct();
        pwm1.set_top(MMD_PWM_TOP);
        pwm1.enable();
        mmd_bl2_ctl_channel!(pwm1).output_to(mmd_spd_ctrl_bl1!(pins));

        let dispenser0_motor = BrushlessMotor::new(
            mmd_dir_bl0!(pins).into_push_pull_output().into_dyn_pin(),
            pwm0,
            MmdBrushlessMotor0Channel,
            (0.03, 0.45, 0.55, 0.97),
            false,
        );
        let dispenser1_motor = BrushlessMotor::new(
            mmd_dir_bl1!(pins).into_push_pull_output().into_dyn_pin(),
            pwm1,
            MmdBrushlessMotor1Channel,
            (0.03, 0.45, 0.55, 0.97),
            false,
        );
        let brushless_motor_processor =
            BrushlessMotorProcessor::new(dispenser0_motor, dispenser1_motor);

        unsafe {
            BRUSHLESS_MOTOR_PROCESSOR = Some(brushless_motor_processor);
        }
    }

    {
        let mut pwm = mmd_br0_pwm_slice!(pwm_slices);
        pwm.set_ph_correct();
        pwm.set_top(MMD_PWM_TOP);

        pwm.enable();
        mmd_br_channel_a!(pwm).output_to(mmd_br_pwm_a!(pins));
        //mmd_br_channel_b!(pwm).output_to(mmd_br_pwm_b!(pins));
        //mmd_br_channel_b!(pwm).set_inverted();

        let peristaltic_pump_motor = BrushMotorPatch::new(
            mmd_br_nEN!(pins).into_push_pull_output().into_dyn_pin(),
            mmd_br_pwm_b!(pins).into_push_pull_output().into_dyn_pin(), // pwmb as dir pin
            pwm,
            MmdBrushMotorChannel,
            (0.03, 0.45, 0.55, 0.97),
            false,
        );
        let brush_motor_processor = BrushMotorProcessor::new(peristaltic_pump_motor);
        unsafe {
            BRUSH_MOTOR_PROCESSOR = Some(brush_motor_processor);
        }
    }

    // init tmc pin
    let mut tmc_uart = mmd_tmc_uart_tx!(pins)
        .into_pull_down_disabled()
        .into_push_pull_output();
    tmc_uart.set_low().unwrap();

    info!("hello world");
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
                MMD_STEPPER42_0_REVERT_DIR,
            ),
            left_limit_pin,
            right_limit_pin,
        ));
        spawn_task(process_mmd_linear_stepper_message(processor));
    }
    {
        // 使用第二个通道连接驱动传送带旋转的电机
        let enable_pin_42 = mmd_stepper42_nEN1!(pins);
        let dir_pin_42 = mmd_stepper42_dir1!(pins)
            .into_push_pull_output()
            .into_dyn_pin();
        //let step_pin_42 = mmd_stepper42_step1!(pins).into_push_pull_output();

        let mut pwm_42 = mmd_motor42_pwm_slice1!(pwm_slices);
        mmd_motor42_step1_channel!(pwm_42).output_to(mmd_stepper42_step1!(pins));

        // init 57
        let enable_pin_57 = Some(
            mmd_stepper57_nEN!(pins)
                .into_push_pull_output()
                .into_dyn_pin(),
        );
        let dir_pin_57 = mmd_stepper57_dir!(pins)
            .into_push_pull_output()
            .into_dyn_pin();
        //let step_pin_57 = mmd_stepper57_step!(pins).into_push_pull_output();

        let mut pwm_57 = mmd_stepper57_pwm_slice!(pwm_slices);
        mmd_stepper57_step_channel!(pwm_57).output_to(mmd_stepper57_step!(pins));

        let processor = RotationStepperProcessor::new(
            PwmStepper::new(
                enable_pin_42,
                dir_pin_42,
                MmdMotor42Step1Channel,
                clocks.peripheral_clock.freq(),
                pwm_42,
                200, // 无细分，一圈200脉冲
                MMD_STEPPER42_1_REVERT_DIR,
            ),
            PwmStepper::new(
                enable_pin_57,
                dir_pin_57,
                MmdMotor57StepChannel,
                clocks.peripheral_clock.freq(),
                pwm_57,
                200, // 无细分，一圈200脉冲
                MMD_STEPPER57_REVERT_DIR,
            ),
        );
        spawn_task(process_mmd_rotation_stepper_message(processor));
    }

    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

// MMD main future
async fn mmd_process_messages() {
    info!("[MMD] mmd_process_messages 0");
    let (uart, uart_dir) = unsafe { UART.as_mut().unwrap() };
    let mut uart_comm = UartComm::new(uart, uart_dir, UART_EXPECTED_RESPONSE_LENGTH);
    let mut mmd_linear_stepper_available = true;
    let brushless_motor_processor = unsafe { BRUSHLESS_MOTOR_PROCESSOR.as_mut().unwrap() };
    let brush_motor_processor = unsafe { BRUSH_MOTOR_PROCESSOR.as_mut().unwrap() };
    loop {
        if let Some(message) = get_mq().dequeue() {
            info!(
                "[MMD] process_messages() 1.1 | dequeued message: {}",
                message
            );

            // 处理消息
            let res = match message {
                AtomiProto::Mmd(MmdCommand::MmdPing) => {
                    uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdPong))
                }

                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::WaitIdle)) => {
                    // // 处理wait idle 不能受 mmd_linear_stepper_available限制，先用这个办法workaround掉
                    // let res = uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdAck));
                    // linear_stepper_input_mq().enqueue(LinearStepperCommand::WaitIdle);
                    // res

                    if mmd_linear_stepper_available {
                        uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdAck))
                    } else {
                        let _ = uart_comm.send(AtomiProto::AtomiError(AtomiError::MmdUnavailable(
                            MmdStatus::Unavailable,
                        )));
                        Err(AtomiError::MmdUnavailable(MmdStatus::Unavailable))
                    }
                }

                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(cmd)) => {
                    if mmd_linear_stepper_available {
                        let res = uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdAck));
                        linear_stepper_input_mq().enqueue(cmd);
                        mmd_linear_stepper_available = false;
                        res
                    } else {
                        let _ = uart_comm.send(AtomiProto::AtomiError(AtomiError::MmdUnavailable(
                            MmdStatus::Unavailable,
                        )));
                        Err(AtomiError::MmdUnavailable(MmdStatus::Unavailable))
                    }
                }

                AtomiProto::Mmd(MmdCommand::MmdRotationStepper(cmd)) => {
                    // info!("pre rotation");
                    rotation_stepper_input_mq().enqueue(cmd);
                    // info!("send ack 0");
                    uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdAck))
                }

                AtomiProto::Mmd(MmdCommand::MmdDisperser(cmd)) => {
                    brushless_motor_processor.process(cmd).unwrap();
                    // info!("send ack 1");
                    uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdAck))
                }

                AtomiProto::Mmd(MmdCommand::MmdPeristalticPump(cmd)) => {
                    brush_motor_processor.process(cmd).unwrap();
                    // info!("send ack 2");
                    uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdAck))
                }

                _ => Err(AtomiError::IgnoredMsg), // Ignore unrelated commands
            };

            if let Err(err) = res {
                info!("[MMD] message processing error: {}", err);
                continue;
            }
        }

        if let Some(linear_stepper_resp) = linear_stepper_output_mq().dequeue() {
            info!(
                "[MMD] get response from linear stepper: {}",
                linear_stepper_resp
            );
            mmd_linear_stepper_available = true;
        }
        if let Some(rotation_stepper_resp) = rotation_stepper_output_mq().dequeue() {
            info!(
                "[MMD] get response from rotation stepper: {}",
                rotation_stepper_resp
            );
        }

        // 延迟一段时间
        Delay::new(1.millis()).await;
    }
}

#[interrupt]
unsafe fn UART1_IRQ() {
    //info!("UART1_IRQ 0");
    if let Some((uart, uart_dir)) = UART.as_mut() {
        // TODO(zephyr): Move the variable below to global static.

        // 读取一个字节以确定消息长度
        let mut length_buffer = [0; 1];
        //        info!("uart length = {}", length_buffer);
        // info!(
        //     "uart dir state = high?: {}",
        //     uart_dir.as_ref().expect("wtf").is_set_high()
        // );
        if uart.read_full_blocking(&mut length_buffer).is_err() {
            debug!("Errors in reading UART in first byte");
            return;
        }

        let message_length = length_buffer[0] as usize;
        let mut message_buffer = vec![0; message_length];
        if uart.read_full_blocking(&mut message_buffer).is_err() {
            error!(
                "Errors in reading the whole message with size ({})",
                message_length
            );
            return;
        }

        // Parse the message
        info!(
            "{} | UART1_IRQ() 5, len = {}, data: {}",
            now().ticks(),
            message_length,
            Debug2Format(&message_buffer)
        );
        match postcard::from_bytes::<AtomiProto>(&message_buffer) {
            Ok(msg) => {
                info!("Received message: {:?}", msg);
                process_message_in_irq(uart, uart_dir, msg);
            }
            Err(_) => info!("Failed to parse message"),
        }
    }
}

/// 防止mmd process msg的异步函数因为运动被阻塞了，这里在阻塞时，在中端里进行
/// 快速处理。
fn process_message_in_irq(
    uart: &mut MmdUartType,
    uart_dir: &mut Option<MmdUartDirPinType>,
    msg: AtomiProto,
) {
    if let Some(FutureStatus::MmdBusy) = get_status(FutureType::Mmd) {
        if let AtomiProto::Mmd(_) = msg {
            // MMD必须要处理的消息，直接返回busy
            let mut uart_comm = UartComm::new(uart, uart_dir, UART_EXPECTED_RESPONSE_LENGTH);
            if let Err(err) = uart_comm.send(AtomiProto::Mmd(MmdBusy)) {
                error!(
                    "[MMD] Errors in sending MMD BUSY response back to MC, err: {}",
                    err
                );
            }
        }
        return;
    }

    // 正常处理流程
    get_mq().enqueue(msg);
}
