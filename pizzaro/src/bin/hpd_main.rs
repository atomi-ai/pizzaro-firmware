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
use rp2040_hal::gpio::FunctionUart;
use rp2040_hal::multicore::{Multicore, Stack};
use rp2040_hal::uart::{DataBits, StopBits, UartConfig};
use rp_pico::{entry, XOSC_CRYSTAL_FREQ};
use rp_pico::pac::{interrupt};

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{AtomiProto, HpdCommand};
use pizzaro::{hpd_sys_rx, hpd_sys_tx, hpd_uart};
use pizzaro::bsp::{hpd_uart_irq, HpdUart};
use pizzaro::bsp::config::{HPD_BR_THRESHOLD, REVERT_HPD_BR_DIRECTION, REVERT_HPD_LINEARSCALE_DIRECTION};
use pizzaro::common::async_initialization;
use pizzaro::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{Delay, init_global_timer};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::uart_comm::UartComm;
use pizzaro::hpd::hpd_misc::{LinearScale, MOTOR150_PWM_TOP, PwmMotor};
use pizzaro::hpd::linear_bull_processor::{LinearBullProcessor, linear_bull_input_mq, process_linear_bull_message, linear_bull_output_mq};
use pizzaro::hpd::linear_scale::{core1_task, read_and_update_linear_scale};

struct GlobalContainer {
    linear_scale: Option<LinearScale>,
}
static mut GLOBAL_CONTAINER: GlobalContainer = GlobalContainer { linear_scale: None };

static mut UART: Option<HpdUart> = None;

static mut MESSAGE_QUEUE_ONCE: Once<MessageQueueWrapper<AtomiProto>> = Once::new();
fn get_mq() -> &'static mut MessageQueueWrapper<AtomiProto> {
    unsafe { MESSAGE_QUEUE_ONCE.get_mut() }
}
static mut CORE1_STACK: Stack<4096> = Stack::new();

#[entry]
fn main() -> ! {
    async_initialization();
    let mut pac = pac::Peripherals::take().unwrap();
    let mut sio = Sio::new(pac.SIO);
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

    {
        let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
        let cores = mc.cores();
        let core1 = &mut cores[1];
        let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            core1_task()
        });

        unsafe {
            GLOBAL_CONTAINER.linear_scale.replace(LinearScale::new());
        }
        let linear_scale_rc0 = unsafe { GLOBAL_CONTAINER.linear_scale.as_mut().unwrap() };
        let linear_scale_rc1 = unsafe { GLOBAL_CONTAINER.linear_scale.as_mut().unwrap() };
        {
            // Start scale
            spawn_task(read_and_update_linear_scale(sio.fifo, linear_scale_rc0));
        }
        // Start motor150
        let pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
        let mut pwm = pwm_slices.pwm0;
        pwm.set_ph_correct();
        pwm.set_top(MOTOR150_PWM_TOP);
        pwm.enable();
        pwm.channel_a.output_to(pins.gpio16);
        pwm.channel_b.output_to(pins.gpio17);
        pwm.channel_b.set_inverted();

        let processor = LinearBullProcessor::new(
            linear_scale_rc1,
            PwmMotor::new(
                pins.gpio18.into_push_pull_output().into_dyn_pin(),
                pwm,
                HPD_BR_THRESHOLD,
                REVERT_HPD_BR_DIRECTION,
                REVERT_HPD_LINEARSCALE_DIRECTION,
            ),
        );

        spawn_task(process_linear_bull_message(processor));
        spawn_task(hpd_process_messages());
    }

    {
        let uart_pins = (
            hpd_sys_tx!(pins).into_function::<FunctionUart>(), // TX, not used in this program
            hpd_sys_rx!(pins).into_function::<FunctionUart>(), // RX
        );
        let mut uart = UartPeripheral::new(hpd_uart!(pac), uart_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        uart.enable_rx_interrupt();
        unsafe {
            UART = Some(uart);
            NVIC::unmask(hpd_uart_irq());
        }
    }

    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

async fn hpd_process_messages() {
    let uart = unsafe { UART.as_mut().unwrap() };
    let mut uart_comm = UartComm::new(uart, UART_EXPECTED_RESPONSE_LENGTH);
    let mut linear_bull_available = true;
    loop {
        Delay::new(1.millis()).await;

        if let Some(message) = get_mq().dequeue() {
            info!("[HPD] process_messages() 1.1 | dequeued message: {}", message);
            // 处理消息
            let res = match message {
                AtomiProto::Hpd(HpdCommand::HpdPing) => {
                    uart_comm.send(AtomiProto::Hpd(HpdCommand::HpdPong))
                }
                AtomiProto::Hpd(HpdCommand::HpdLinearBull(cmd)) => {
                    if linear_bull_available {
                        let res = uart_comm.send(AtomiProto::Hpd(HpdCommand::HpdAck));
                        linear_bull_input_mq().enqueue(cmd);
                        linear_bull_available = false;
                        res
                    } else {
                        let _ = uart_comm.send(AtomiProto::AtomiError(AtomiError::HpdUnavailable));
                        Err(AtomiError::HpdUnavailable)
                    }
                },
                _ => Err(AtomiError::IgnoredMsg), // Ignore unrelated commands
            };
            if let Err(err) = res {
                info!("[HPD] message processing error: {}", err);
            }
        }

        if let Some(linear_bull_resp) = linear_bull_output_mq().dequeue() {
            info!("[MMD] get response from linear bull: {}", linear_bull_resp);
            linear_bull_available = true;
        }
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