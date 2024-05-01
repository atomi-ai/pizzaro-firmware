// Detach Unit => DTU
// 不沾铲，解决面饼吸附在顶层的问题
#![no_std]
#![no_main]
extern crate alloc;

use alloc::boxed::Box;
use alloc::vec;
use core::sync::atomic::Ordering;
use cortex_m::asm::delay;
use cortex_m::peripheral::NVIC;
use defmt::{debug, error, info, Debug2Format};
use fugit::{ExtU64, RateExtU32};
use rp2040_hal::gpio::{FunctionUart, PullUp};
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
use generic::atomi_proto::DtuCommand::{self};
use generic::atomi_proto::{AtomiProto, StepperCommand, StepperResponse};
use pizzaro::bsp::board_dtu_release_sb::{dtu_uart_irq, DtuUartDirPinType, DtuUartType};
use pizzaro::common::async_initialization;
use pizzaro::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, now, Delay, DelayCreator};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::stepper::classic_stepper::Stepper;
use pizzaro::common::stepper::classic_stepper_driver::StepperDriver;
use pizzaro::common::stepper::classic_stepper_processor::{
    process_classic_stepper_message, DualQueue, LinearStepperProcessor,
};
use pizzaro::common::stepper::{init_tmc_parameters, GLOBAL_STEPPER_STOP};
use pizzaro::common::uart_comm::UartComm;
use pizzaro::{
    dtu_485_dir, dtu_limit0, dtu_limit1, dtu_stepper_dir, dtu_stepper_n_en, dtu_stepper_step,
    dtu_sys_rx, dtu_sys_tx, dtu_tmc_uart_rx, dtu_tmc_uart_tx, dtu_uart,
};

// TODO(zephyr): Move these global static into a GlobalStatic struct.
static mut UART: Option<(DtuUartType, Option<DtuUartDirPinType>)> = None;

static mut MESSAGE_QUEUE_ONCE: Once<MessageQueueWrapper<AtomiProto>> = Once::new();
fn get_dtu_mq() -> &'static mut MessageQueueWrapper<AtomiProto> {
    unsafe { MESSAGE_QUEUE_ONCE.get_mut() }
}

static mut DTU_DUAL_QUEUE_ONCE: Once<DualQueue> = Once::new();
fn get_dtu_dual_queue() -> &'static mut DualQueue {
    unsafe { DTU_DUAL_QUEUE_ONCE.get_mut() }
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

    let pins =
        rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let uart_pins = (
        dtu_sys_tx!(pins).into_function::<FunctionUart>(), // TX, not used in this program
        dtu_sys_rx!(pins).into_function::<FunctionUart>(), // RX
    );
    let uart_dir = dtu_485_dir!(pins).reconfigure();

    let mut uart = UartPeripheral::new(dtu_uart!(pac), uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.enable_rx_interrupt();
    unsafe {
        UART = Some((uart, Some(uart_dir)));
        NVIC::unmask(dtu_uart_irq());
    }

    debug!("hello world");
    {
        spawn_task(dtu_process_messages());
    }
    {
        let tmc_2209_uart_pins = (
            dtu_tmc_uart_tx!(pins).reconfigure::<FunctionUart, PullUp>(),
            dtu_tmc_uart_rx!(pins).reconfigure::<FunctionUart, PullUp>(),
        );
        let tmc_2209_uart = UartPeripheral::new(pac.UART1, tmc_2209_uart_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();
        let (tmc_rx, tmc_tx) = tmc_2209_uart.split();
        init_tmc_parameters(25, 5, Some(100), 0, tmc_rx, tmc_tx).expect("init tmc error");

        // 使用第一个通道连接驱动伸缩的电机
        let enable_pin = dtu_stepper_n_en!(pins).into_push_pull_output();
        let dir_pin = dtu_stepper_dir!(pins).into_push_pull_output();
        let step_pin = dtu_stepper_step!(pins).into_push_pull_output();
        let delay_creator = DelayCreator::new();
        let left_limit_pin = dtu_limit0!(pins).into_pull_down_input();
        let right_limit_pin = dtu_limit1!(pins).into_pull_down_input();

        let driver = StepperDriver::new(enable_pin, dir_pin, step_pin, delay_creator, true);
        let stepper = Stepper::new(driver, left_limit_pin, right_limit_pin);
        let processor = LinearStepperProcessor::new(stepper);
        spawn_task(process_classic_stepper_message(processor, get_dtu_dual_queue()));
    }

    start_global_executor();
    GLOBAL_STEPPER_STOP.store(false, Ordering::Relaxed);

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

// DTU main future
async fn dtu_process_messages() {
    debug!("[DTU] dtu_process_messages 0");
    let (uart, uart_dir) = unsafe { UART.as_mut().unwrap() };
    let mut uart_comm = UartComm::new(uart, uart_dir, UART_EXPECTED_RESPONSE_LENGTH);
    let mut dtu_stepper_available = true;
    loop {
        if let Some(message) = get_dtu_mq().dequeue() {
            info!("[DTU] process_messages() 1.1 | dequeued message: {}", message);

            // 处理消息
            let res = match message {
                AtomiProto::Dtu(DtuCommand::DtuPing) => {
                    uart_comm.send(AtomiProto::Dtu(DtuCommand::DtuPong))
                }

                AtomiProto::Dtu(DtuCommand::DtuLinear(StepperCommand::WaitIdle)) => {
                    // // 处理wait idle 不能受 dtu_stepper_available限制，先用这个办法workaround掉
                    // let res = uart_comm.send(AtomiProto::Mmd(MmdCommand::MmdAck));
                    // stepper_input_mq().enqueue(LinearStepperCommand::WaitIdle);
                    // res

                    if dtu_stepper_available {
                        uart_comm.send(AtomiProto::Dtu(DtuCommand::DtuAck))
                    } else {
                        let _ = uart_comm.send(AtomiProto::AtomiError(AtomiError::DtuUnavailable));
                        Err(AtomiError::DtuUnavailable)
                    }
                }

                AtomiProto::Dtu(DtuCommand::DtuLinear(cmd)) => {
                    if dtu_stepper_available {
                        let res = uart_comm.send(AtomiProto::Dtu(DtuCommand::DtuAck));
                        get_dtu_dual_queue().push_request(cmd);
                        dtu_stepper_available = false;
                        res
                    } else {
                        let _ = uart_comm.send(AtomiProto::AtomiError(AtomiError::DtuUnavailable));
                        Err(AtomiError::DtuUnavailable)
                    }
                }

                AtomiProto::Dtu(DtuCommand::DtuStop) => {
                    if !dtu_stepper_available {
                        // If stepper is running, stop it directly.
                        GLOBAL_STEPPER_STOP.store(true, Ordering::Relaxed);
                    }

                    if !dtu_stepper_available {
                        // wait till the stepper is stopped.
                        loop {
                            if let Some(stepper_resp) = get_dtu_dual_queue().pop_response() {
                                info!("stepper_resp: {}", stepper_resp);
                                assert_eq!(
                                    stepper_resp,
                                    StepperResponse::Error(AtomiError::DtuStopped)
                                );
                                break;
                            }
                        }
                        GLOBAL_STEPPER_STOP.store(false, Ordering::Relaxed);
                        dtu_stepper_available = true;
                    }
                    uart_comm.send(AtomiProto::Dtu(DtuCommand::DtuAck))
                }

                _ => Err(AtomiError::IgnoredMsg), // Ignore unrelated commands
            };

            if let Err(err) = res {
                info!("[DTU] message processing error: {}", err);
                continue;
            }
        }

        if let Some(stepper_resp) = get_dtu_dual_queue().pop_response() {
            info!("[DTU] get response from linear stepper: {}", stepper_resp);
            dtu_stepper_available = true;
        }

        // 延迟一段时间
        Delay::new(1.millis()).await;
    }
}

#[interrupt]
unsafe fn UART0_IRQ() {
    //info!("UART1_IRQ 0");
    if let Some((uart, _uart_dir)) = UART.as_mut() {
        // TODO(zephyr): Move the variable below to global static.

        // 读取一个字节以确定消息长度
        let mut length_buffer = [0; 1];
        if uart.read_full_blocking(&mut length_buffer).is_err() {
            debug!("Errors in reading UART in first byte");
            return;
        }

        let message_length = length_buffer[0] as usize;
        let mut message_buffer = vec![0; message_length];
        if uart.read_full_blocking(&mut message_buffer).is_err() {
            error!("Errors in reading the whole message with size ({})", message_length);
            return;
        }

        // Parse the message
        debug!(
            "{} | UART0_IRQ() 5, len = {}, data: {}",
            now().ticks(),
            message_length,
            Debug2Format(&message_buffer)
        );
        match postcard::from_bytes::<AtomiProto>(&message_buffer) {
            Ok(msg) => {
                debug!("Received message: {:?}", msg);
                get_dtu_mq().enqueue(msg);
            }
            Err(_) => info!("Failed to parse message"),
        }
    }
}
