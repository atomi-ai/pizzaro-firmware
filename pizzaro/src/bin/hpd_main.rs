#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec;

use cortex_m::asm::delay;
use cortex_m::peripheral::NVIC;
use defmt::{Debug2Format, error, info};
use fugit::{ExtU64, RateExtU32};
use rp2040_hal::{clocks::{Clock, init_clocks_and_plls}, pac, sio::Sio, Timer, uart::UartPeripheral, watchdog::Watchdog};
use rp2040_hal::gpio::{FunctionUart, Pin, PullDown};
use rp2040_hal::gpio::bank0::{Gpio8, Gpio9};
use rp2040_hal::uart::{DataBits, Enabled, StopBits, UartConfig};
use rp_pico::{entry, XOSC_CRYSTAL_FREQ};
use rp_pico::pac::{interrupt, UART1};

use generic::atomi_error::AtomiError;
use generic::atomi_proto::AtomiProto;
use pizzaro::common::async_initialization;
use pizzaro::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{Delay, init_global_timer};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::uart_comm::UartComm;
use pizzaro::hpd_process::process_hpd_message;
use pizzaro::message_queue::{MessageQueueInterface, MessageQueueWrapper};

static mut UART: Option<UartPeripheral<Enabled, UART1, (
    Pin<Gpio8, FunctionUart, PullDown>, Pin<Gpio9, FunctionUart, PullDown>)>> = None;

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
        pins.gpio8.into_function::<FunctionUart>(), // TX, not used in this program
        pins.gpio9.into_function::<FunctionUart>(), // RX
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
        NVIC::unmask(pac::Interrupt::UART1_IRQ);
    }

    spawn_task(process_messages());
    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

async fn process_messages() {
    loop {
        if let Some(message) = get_mq().dequeue() {
            info!("[HPD] process_messages() 1.1 | dequeued message: {}", message);
            // 处理消息
            let resp = match message {
                AtomiProto::Hpd(msg) => process_hpd_message(msg),
                _ => Err(AtomiError::IgnoredMsg), // Ignore unrelated commands
            };
            let resp_cmd = match resp {
                Ok(cmd) => cmd,
                Err(err) => {
                    info!("[HPD] msg processing error: {}", err);
                    continue;
                }
            };
            let uart = unsafe { UART.as_mut().unwrap() };
            let mut uart_comm = UartComm::new(uart, UART_EXPECTED_RESPONSE_LENGTH);
            let res = uart_comm.send(AtomiProto::Hpd(resp_cmd));
            match res {
                Ok(data) => info!("[HPD] Sent resp {:?} and result: {:?}", resp_cmd, data),
                Err(err) => error!("[HPD] Sent resp {:?} and got error: {:?}", resp_cmd, err),
            }
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

        info!("UART1_IRQ() 0");
        // 读取一个字节以确定消息长度
        let mut length_buffer = [0; 1];
        if uart.read_full_blocking(&mut length_buffer).is_err() {
            error!("Errors in reading UART");
            return;
        }

        info!("UART1_IRQ() 3");
        let message_length = length_buffer[0] as usize;
        let mut message_buffer = vec![0; message_length];
        if uart.read_full_blocking(&mut message_buffer).is_err() {
            error!("Errors in reading the whole message with size ({})", message_length);
            return;
        }

        // Parse the message
        info!("UART1_IRQ() 5, len = {}, data: {}", message_length, Debug2Format(&message_buffer));
        match postcard::from_bytes::<AtomiProto>(&message_buffer) {
            Ok(message) => {
                info!("Received message: {:?}", message);
                get_mq().enqueue(message);
            },
            Err(_) => info!("Failed to parse message"),
        }
        info!("UART1_IRQ() 9");
    }
}
