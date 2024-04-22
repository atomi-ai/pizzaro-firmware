/*
   In this version, we use CAN-bus for inter-mcu communication.
*/
#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use core::sync::atomic::Ordering;

use can2040::{Can2040, CanFrame};
use cortex_m::asm::delay;
use defmt::{debug, error, info, Debug2Format, Format};
use embedded_can::nb::Can;
use embedded_can::{Frame, Id, StandardId};
use fugit::ExtU64;
use rp2040_hal::multicore::{Multicore, Stack};
use rp2040_hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog, Timer};
use rp_pico::{entry, XOSC_CRYSTAL_FREQ};
use serde::Serialize;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{AtomiErrorWithCanId, HpdCommand, LinearBullCommand, LinearBullResponse};
use generic::mmd_status::MmdStatus;
use pizzaro::bsp::config::{
    HPD_BR_DRIVER_N_EN, HPD_BR_THRESHOLD, HPD_MOTOR150_PWM_TOP, REVERT_HPD_BR_DIRECTION,
};
use pizzaro::common::async_initialization;
use pizzaro::common::brush_motor::BrushMotor;
use pizzaro::common::can::{
    enqueue_can_frame_if_possible, get_can, init_can_bus, send_can_message,
};
use pizzaro::common::consts::{CANBUS_FREQUENCY, HPD_CAN_ID, MC_CAN_ID, MMD_CAN_ID};
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, Delay};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::hpd::hpd_misc::LinearScale;
use pizzaro::hpd::linear_bull_processor::{
    linear_bull_input_mq, linear_bull_output_mq, process_linear_bull_message, LinearBullProcessor,
};
use pizzaro::hpd::linear_scale::{core1_task, read_and_update_linear_scale};
use pizzaro::hpd::GLOBAL_LINEAR_BULL_STOP;
use pizzaro::{hpd_br_nEN, hpd_br_pwm_a, hpd_br_pwm_b};
use pizzaro::{hpd_sys_rx, hpd_sys_tx};

struct GlobalContainer {
    linear_scale: Option<LinearScale>,
}
static mut GLOBAL_CONTAINER: GlobalContainer = GlobalContainer { linear_scale: None };

static mut HPD_MESSAGE_QUEUE_ONCE: Once<MessageQueueWrapper<HpdCommand>> = Once::new();
fn get_hpd_mq() -> &'static mut MessageQueueWrapper<HpdCommand> {
    unsafe { HPD_MESSAGE_QUEUE_ONCE.get_mut() }
}
static mut CORE1_STACK: Stack<4096> = Stack::new();

#[entry]
fn main() -> ! {
    async_initialization();
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();
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
        // init CAN bus here.
        init_can_bus(
            &mut core,
            CANBUS_FREQUENCY,
            hpd_sys_rx!(pins).id().num as u32,
            hpd_sys_tx!(pins).id().num as u32,
        );
        spawn_task(process_can_hpd_message());
    }

    start_global_executor();
    GLOBAL_LINEAR_BULL_STOP.store(false, Ordering::Relaxed);
    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

async fn hpd_process_messages() {
    let mut linear_bull_available = true;
    loop {
        Delay::new(1.millis()).await;

        if let Some(message) = get_hpd_mq().dequeue() {
            info!("[HPD] process_messages() 1.1 | dequeued message: {}", message);
            // 处理消息
            let res = match message {
                HpdCommand::HpdPing => send_can_message(MC_CAN_ID, HpdCommand::HpdPong),

                HpdCommand::HpdStop => {
                    if !linear_bull_available {
                        GLOBAL_LINEAR_BULL_STOP.store(true, Ordering::Relaxed);
                        loop {
                            if let Some(resp) = linear_bull_output_mq().dequeue() {
                                info!("linear bull resp: {}", resp);
                                assert_eq!(resp, LinearBullResponse::Error(AtomiError::HpdStopped));
                                break;
                            }
                        }
                        linear_bull_available = true;
                        GLOBAL_LINEAR_BULL_STOP.store(false, Ordering::Relaxed);
                    }
                    send_can_message(MC_CAN_ID, HpdCommand::HpdAck)
                }

                HpdCommand::HpdLinearBull(LinearBullCommand::WaitIdle) => {
                    if linear_bull_available {
                        send_can_message(MC_CAN_ID, HpdCommand::HpdAck)
                    } else {
                        let _ = send_can_message(MC_CAN_ID, AtomiErrorWithCanId::new(
                            HPD_CAN_ID, AtomiError::HpdUnavailable));
                        Err(AtomiError::HpdUnavailable)
                    }
                }

                HpdCommand::HpdLinearBull(cmd) => {
                    if linear_bull_available {
                        let res = send_can_message(MC_CAN_ID, HpdCommand::HpdAck);
                        linear_bull_input_mq().enqueue(cmd);
                        linear_bull_available = false;
                        res
                    } else {
                        let _ = send_can_message(MC_CAN_ID, AtomiErrorWithCanId::new(
                            HPD_CAN_ID, AtomiError::HpdUnavailable));
                        Err(AtomiError::HpdUnavailable)
                    }
                }
                _ => Err(AtomiError::IgnoredMsg), // Ignore unrelated commands
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

async fn process_can_hpd_message() {
    loop {
        Delay::new(100.micros()).await;

        match get_can().receive() {
            Ok(f) => {
                enqueue_can_frame_if_possible(HPD_CAN_ID, f, get_hpd_mq());
            }
            Err(nb::Error::Other(err)) => {
                error!("Canbus error: {}", err);
            }
            Err(_) => (),
        }
    }
}
