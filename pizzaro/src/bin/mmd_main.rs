#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec;

use cortex_m::asm::delay;
use cortex_m::peripheral::NVIC;
use defmt::{Debug2Format, error, info};
use fugit::{ExtU64, RateExtU32};
use rp2040_hal::{
    clocks::{Clock, init_clocks_and_plls},
    pac,
    sio::Sio,
    Timer,
    uart::UartPeripheral,
    watchdog::Watchdog,
};
use rp2040_hal::gpio::FunctionUart;
use rp2040_hal::uart::{DataBits, StopBits, UartConfig};
use rp_pico::{entry, XOSC_CRYSTAL_FREQ};
use rp_pico::pac::interrupt;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{AtomiProto, MmdCommand};
use generic::atomi_proto::MmdCommand::MmdBusy;
use generic::mmd_status::MmdStatus;
use pizzaro::{
    mmd_limit0, mmd_limit1, mmd_stepper42_dir0, mmd_stepper42_nEN0, mmd_stepper42_step0,
};
use pizzaro::{common::async_initialization, mmd_sys_rx, mmd_sys_tx};
use pizzaro::bsp::mmd_uart_irq;
use pizzaro::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_status::{FutureStatus, FutureType, get_status};
use pizzaro::common::global_timer::{Delay, DelayCreator, init_global_timer, now};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::uart_comm::UartComm;
use pizzaro::mmd::linear_stepper::LinearStepper;
use pizzaro::mmd::linear_stepper_processor::{linear_stepper_input_mq, linear_stepper_output_mq, LinearStepperProcessor, process_mmd_linear_stepper_message};
use pizzaro::mmd::mmd_dispatcher::MmdUartType;
use pizzaro::mmd::stepper::Stepper;

static mut UART: Option<MmdUartType> = None;

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

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let uart_pins = (
        mmd_sys_tx!(pins).into_function::<FunctionUart>(), // TX, not used in this program
        mmd_sys_rx!(pins).into_function::<FunctionUart>(), // RX
    );
    let mut uart = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.enable_rx_interrupt();
    unsafe {
        UART = Some(uart);
        NVIC::unmask(mmd_uart_irq());
    }

    info!("hello world");
    {
        let enable_pin = mmd_stepper42_nEN0!(pins).into_push_pull_output();
        let dir_pin = mmd_stepper42_dir0!(pins).into_push_pull_output();
        let step_pin = mmd_stepper42_step0!(pins).into_push_pull_output();
        let left_limit_pin = mmd_limit0!(pins).into_pull_down_input();
        let right_limit_pin = mmd_limit1!(pins).into_pull_down_input();
        let delay_creator = DelayCreator::new();
        let processor = LinearStepperProcessor::new(LinearStepper::new(
            Stepper::new(enable_pin, dir_pin, step_pin, delay_creator),
            left_limit_pin,
            right_limit_pin,
        ));
        spawn_task(mmd_process_messages());
        spawn_task(process_mmd_linear_stepper_message(processor));
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
    let uart = unsafe { UART.as_mut().unwrap() };
    let mut uart_comm = UartComm::new(uart, UART_EXPECTED_RESPONSE_LENGTH);
    let mut mmd_available = true;
    loop {
        if let Some(message) = get_mq().dequeue() {
            info!("[MMD] process_messages() 1.1 | dequeued message: {}", message);

            // 处理消息
            let res = match message {
                AtomiProto::Mmd(MmdCommand::MmdPing) => {
                    uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdPong))
                }
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(cmd)) => {
                    if mmd_available {
                        let res = uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdAck));
                        linear_stepper_input_mq().enqueue(cmd);
                        mmd_available = false;
                        res
                    } else {
                        let _ = uart_comm.send(AtomiProto::AtomiError(
                            AtomiError::MmdUnavailable(MmdStatus::Unavailable)));
                        Err(AtomiError::MmdUnavailable(MmdStatus::Unavailable))
                    }
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
            mmd_available = true;
        }

        // 延迟一段时间
        Delay::new(1.millis()).await;
    }
}

#[interrupt]
unsafe fn UART1_IRQ() {
    info!("UART1_IRQ 0");
    if let Some(uart) = UART.as_mut() {
        // TODO(zephyr): Move the variable below to global static.

        // 读取一个字节以确定消息长度
        let mut length_buffer = [0; 1];
        info!("uart length = {}", length_buffer);
        if uart.read_full_blocking(&mut length_buffer).is_err() {
            error!("Errors in reading UART");
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
                process_message_in_irq(uart, msg);
            }
            Err(_) => info!("Failed to parse message"),
        }
    }
}

/// 防止mmd process msg的异步函数因为运动被阻塞了，这里在阻塞时，在中端里进行
/// 快速处理。
fn process_message_in_irq(uart: &mut MmdUartType, msg: AtomiProto) {
    if let Some(FutureStatus::MmdBusy) = get_status(FutureType::Mmd) {
        if let AtomiProto::Mmd(_) = msg {
            // MMD必须要处理的消息，直接返回busy
            let mut uart_comm = UartComm::new(uart, UART_EXPECTED_RESPONSE_LENGTH);
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
