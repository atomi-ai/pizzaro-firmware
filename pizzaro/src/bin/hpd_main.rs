#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec;
use core::sync::atomic::Ordering;

use cortex_m::asm::delay;
use cortex_m::peripheral::NVIC;
use defmt::{debug, error, info, warn, Debug2Format};
use fugit::{ExtU64, RateExtU32};
use rp2040_hal::gpio::FunctionUart;
use rp2040_hal::multicore::{Multicore, Stack};
use rp2040_hal::uart::{DataBits, StopBits, UartConfig};
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    uart::UartPeripheral,
    watchdog::Watchdog,
    Timer,
};
use rp_pico::pac::interrupt;
use rp_pico::{entry, XOSC_CRYSTAL_FREQ};

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{AtomiProto, HpdCommand, LinearBullCommand, LinearBullResponse};
use pizzaro::bsp::board_hpd_release_sb::{hpd_uart_irq, HpdUartType};
use pizzaro::bsp::config::{
    HPD_BR_DRIVER_N_EN, HPD_BR_THRESHOLD, HPD_MOTOR150_PWM_TOP, REVERT_HPD_BR_DIRECTION,
};
use pizzaro::common::async_initialization;
use pizzaro::common::brush_motor::BrushMotor;
use pizzaro::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, Delay};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::uart_comm::UartComm;
use pizzaro::hpd::hpd_misc::LinearScale;
use pizzaro::hpd::linear_bull_processor::{
    linear_bull_input_mq, linear_bull_output_mq, process_linear_bull_message, LinearBullProcessor,
};
use pizzaro::hpd::linear_scale::{core1_task, read_and_update_linear_scale};
use pizzaro::hpd::GLOBAL_LINEAR_BULL_STOP;
use pizzaro::{
    bsp::board_hpd_release_sb::HpdUartDirPinType, hpd_485_dir, hpd_sys_rx, hpd_sys_tx, hpd_uart,
};
use pizzaro::{hpd_br_nEN, hpd_br_pwm_a, hpd_br_pwm_b};

struct GlobalContainer {
    linear_scale: Option<LinearScale>,
}
static mut GLOBAL_CONTAINER: GlobalContainer = GlobalContainer { linear_scale: None };

static mut UART: Option<(HpdUartType, Option<HpdUartDirPinType>)> = None;

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

    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    {
        let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
        let cores = mc.cores();
        let core1 = &mut cores[1];
        let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || core1_task());

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
        pwm.set_top(HPD_MOTOR150_PWM_TOP);
        pwm.enable();
        // pwm.channel_a.output_to(pins.gpio16);
        // pwm.channel_b.output_to(pins.gpio17);
        pwm.channel_a.output_to(hpd_br_pwm_a!(pins));
        pwm.channel_b.output_to(hpd_br_pwm_b!(pins));
        pwm.channel_b.set_inverted();

        let processor = LinearBullProcessor::new(
            linear_scale_rc1,
            BrushMotor::new(
                hpd_br_nEN!(pins).into_push_pull_output().into_dyn_pin(),
                pwm,
                HPD_BR_THRESHOLD,
                REVERT_HPD_BR_DIRECTION,
                HPD_BR_DRIVER_N_EN,
                HPD_MOTOR150_PWM_TOP,
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
        let uart_dir = hpd_485_dir!(pins).reconfigure();

        uart.enable_rx_interrupt();
        unsafe {
            UART = Some((uart, Some(uart_dir)));
            NVIC::unmask(hpd_uart_irq());
        }
    }

    start_global_executor();
    GLOBAL_LINEAR_BULL_STOP.store(false, Ordering::Relaxed);
    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

async fn hpd_process_messages() {
    let (uart, uart_dir) = unsafe { UART.as_mut().unwrap() };
    let mut uart_comm = UartComm::new(uart, uart_dir, UART_EXPECTED_RESPONSE_LENGTH);
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

                AtomiProto::Hpd(HpdCommand::HpdStop) => {
                    //info!("pre stop1, linear_bull_available:{}", linear_bull_available);
                    if !linear_bull_available {
                        GLOBAL_LINEAR_BULL_STOP.store(true, Ordering::Relaxed);
                        //info!("set linear_bull stop");
                        loop {
                            if let Some(resp) = linear_bull_output_mq().dequeue() {
                                //info!("linear bull resp: {}", resp);
                                assert_eq!(resp, LinearBullResponse::Error(AtomiError::HpdStopped));
                                break;
                            }
                            //info!("await");
                            Delay::new(1.millis()).await;
                        }
                        linear_bull_available = true;
                        GLOBAL_LINEAR_BULL_STOP.store(false, Ordering::Relaxed);
                    }
                    uart_comm.send(AtomiProto::Hpd(HpdCommand::HpdAck))
                }

                AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::WaitIdle)) => {
                    if linear_bull_available {
                        uart_comm.send(AtomiProto::Hpd(HpdCommand::HpdAck))
                    } else {
                        let _ = uart_comm.send(AtomiProto::AtomiError(AtomiError::HpdUnavailable));
                        Err(AtomiError::HpdUnavailable)
                    }
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
                }
                unknown => {
                    warn!("message ignored: {}", unknown);
                    Err(AtomiError::IgnoredMsg)
                } // Ignore unrelated commands
            };
            if let Err(err) = res {
                info!("[HPD] message processing error: {}", err);
            }
        }

        if let Some(linear_bull_resp) = linear_bull_output_mq().dequeue() {
            info!("[HPD] get response from linear bull: {}", linear_bull_resp);
            linear_bull_available = true;
        }
    }
}

#[interrupt]
unsafe fn UART1_IRQ() {
    debug!("UART1_IRQ 0");
    if let Some((uart, _uart_dir)) = UART.as_mut() {
        // TODO(zephyr): Move the variable below to global static.

        debug!("UART1_IRQ() 0");
        // 读取一个字节以确定消息长度
        let mut length_buffer = [0; 1];
        if uart.read_full_blocking(&mut length_buffer).is_err() {
            debug!("Errors in reading UART");
            return;
        }

        debug!("UART1_IRQ() 3");
        let message_length = length_buffer[0] as usize;
        let mut message_buffer = vec![0; message_length];
        if uart.read_full_blocking(&mut message_buffer).is_err() {
            error!("Errors in reading the whole message with size ({})", message_length);
            return;
        }

        // Parse the message
        debug!("UART1_IRQ() 5, len = {}, data: {}", message_length, Debug2Format(&message_buffer));
        match postcard::from_bytes::<AtomiProto>(&message_buffer) {
            Ok(message) => {
                info!("Received message: {:?}", message);
                get_mq().enqueue(message);
            }
            Err(_) => info!("Failed to parse message"),
        }
        debug!("UART1_IRQ() 9");
    }
}
