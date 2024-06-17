// Detach Unit => DTU
// 不沾铲，解决面饼吸附在顶层的问题
#![no_std]
#![no_main]
extern crate alloc;

use alloc::boxed::Box;
use alloc::vec;
use core::sync::atomic::AtomicBool;

use cortex_m::asm::delay;
use cortex_m::peripheral::NVIC;
use defmt::{debug, error, info, Debug2Format};
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
use generic::atomi_proto::{AsdCommand, AtomiProto, StepperDriverCommand};
use pizzaro::bsp::board_asd::{asd_uart_irq, AsdUartDirPinType, AsdUartType};
use pizzaro::bsp::config::REVERT_MMD_STEPPER42_0_DIRECTION;
use pizzaro::common::async_initialization;
use pizzaro::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, now, Delay, DelayCreator};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::stepper_driver::StepperDriver;
use pizzaro::common::stepper_driver_processor::{
    process_stepper_driver_message, stepper_driver_input_mq, stepper_driver_output_mq,
    StepperDriverProcessor,
};
use pizzaro::common::uart_comm::UartComm;
use pizzaro::{
    asd_485_dir, asd_stepper_dir, asd_stepper_n_en, asd_stepper_step, asd_sys_rx, asd_sys_tx,
    asd_uart,
};

// TODO(zephyr): Move these global static into a GlobalStatic struct.
// TODO(zephyr): Enable global stop.
pub static GLOBAL_ASD_STEPPER_STOP: AtomicBool = AtomicBool::new(false);

static mut UART: Option<(AsdUartType, Option<AsdUartDirPinType>)> = None;

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

    let pins =
        rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let uart_pins = (
        asd_sys_tx!(pins).into_function::<FunctionUart>(), // TX, not used in this program
        asd_sys_rx!(pins).into_function::<FunctionUart>(), // RX
    );
    let uart_dir = asd_485_dir!(pins).reconfigure();

    let mut uart = UartPeripheral::new(asd_uart!(pac), uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.enable_rx_interrupt();
    unsafe {
        UART = Some((uart, Some(uart_dir)));
        NVIC::unmask(asd_uart_irq());
    }

    debug!("hello world");
    {
        spawn_task(asd_process_messages());
    }
    {
        let enable_pin = asd_stepper_n_en!(pins).into_push_pull_output();
        let dir_pin = asd_stepper_dir!(pins).into_push_pull_output();
        let step_pin = asd_stepper_step!(pins).into_push_pull_output();
        let delay_creator = DelayCreator::new();
        let processor = StepperDriverProcessor::new(StepperDriver::new(
            enable_pin,
            dir_pin,
            step_pin,
            delay_creator,
            REVERT_MMD_STEPPER42_0_DIRECTION,
        ));
        spawn_task(process_stepper_driver_message(processor));
    }

    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

// DTU main future
async fn asd_process_messages() {
    debug!("[DTU] dtu_process_messages 0");
    let (uart, uart_dir) = unsafe { UART.as_mut().unwrap() };
    let mut uart_comm = UartComm::new(uart, uart_dir, UART_EXPECTED_RESPONSE_LENGTH);
    let mut asd_stepper_available = true;
    loop {
        if let Some(message) = get_mq().dequeue() {
            info!("[DTU] process_messages() 1.1 | dequeued message: {}", Debug2Format(&message));

            // 处理消息
            let res = match message {
                AtomiProto::Asd(AsdCommand::AsdPing) => {
                    uart_comm.send(AtomiProto::Asd(AsdCommand::AsdPong))
                }

                AtomiProto::Asd(AsdCommand::AsdStepper(StepperDriverCommand::CheckStatus)) => {
                    if asd_stepper_available {
                        uart_comm.send(AtomiProto::Asd(AsdCommand::AsdAck))
                    } else {
                        let _ = uart_comm.send(AtomiProto::AtomiError(AtomiError::AsdUnavailable));
                        Err(AtomiError::AsdUnavailable)
                    }
                }

                AtomiProto::Asd(AsdCommand::AsdStepper(cmd)) => {
                    if asd_stepper_available {
                        let res = uart_comm.send(AtomiProto::Asd(AsdCommand::AsdAck));
                        stepper_driver_input_mq().enqueue(cmd);
                        asd_stepper_available = false;
                        res
                    } else {
                        let _ = uart_comm.send(AtomiProto::AtomiError(AtomiError::AsdUnavailable));
                        Err(AtomiError::AsdUnavailable)
                    }
                }

                _ => Err(AtomiError::IgnoredMsg), // Ignore unrelated commands
            };

            if let Err(err) = res {
                info!("[DTU] message processing error: {}", Debug2Format(&err));
                continue;
            }
        }

        if let Some(stepper_resp) = stepper_driver_output_mq().dequeue() {
            info!("[DTU] get response from linear stepper: {}", Debug2Format(&stepper_resp));
            asd_stepper_available = true;
        }

        // 延迟一段时间
        Delay::new(1.millis()).await;
    }
}

// TODO(zephyr): The code is same for MMD / HPD / DTU / ASD, let's make it generic.
#[interrupt]
unsafe fn UART1_IRQ() {
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
        debug!("UART1_IRQ() 3: message_length: {}", message_length);
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
                debug!("Received message: {:?}", Debug2Format(&msg));
                get_mq().enqueue(msg);
            }
            Err(_) => info!("Failed to parse message"),
        }
    }
}
