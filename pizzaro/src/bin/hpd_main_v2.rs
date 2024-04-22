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
use defmt::{debug, Debug2Format, error, Format, info};
use embedded_can::{Frame, Id, StandardId};
use embedded_can::nb::Can;
use fugit::{ExtU64};
use rp2040_hal::{
    clocks::{init_clocks_and_plls},
    pac,
    sio::Sio,
    Timer,
    watchdog::Watchdog,
};
use rp2040_hal::multicore::{Multicore, Stack};
use rp_pico::{entry, XOSC_CRYSTAL_FREQ};
use serde::Serialize;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{HpdCommand, LinearBullCommand, LinearBullResponse};
use pizzaro::{hpd_sys_rx, hpd_sys_tx};
use pizzaro::{hpd_br_nEN, hpd_br_pwm_a, hpd_br_pwm_b};
use pizzaro::bsp::config::{
    HPD_BR_DRIVER_N_EN, HPD_BR_THRESHOLD, HPD_MOTOR150_PWM_TOP, REVERT_HPD_BR_DIRECTION,
};
use pizzaro::common::async_initialization;
use pizzaro::common::brush_motor::BrushMotor;
use pizzaro::common::consts::{CANBUS_FREQUENCY, HPD_CAN_ID, MC_CAN_ID};
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{Delay, init_global_timer};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::hpd::GLOBAL_LINEAR_BULL_STOP;
use pizzaro::hpd::hpd_misc::LinearScale;
use pizzaro::hpd::linear_bull_processor::{
    linear_bull_input_mq, linear_bull_output_mq, LinearBullProcessor, process_linear_bull_message,
};
use pizzaro::hpd::linear_scale::{core1_task, read_and_update_linear_scale};

struct GlobalContainer {
    linear_scale: Option<LinearScale>,
}
static mut GLOBAL_CONTAINER: GlobalContainer = GlobalContainer { linear_scale: None };

static mut CAN_BUS_ONCE: Once<Can2040> = Once::new();
fn get_can() -> &'static mut Can2040 {
    unsafe { CAN_BUS_ONCE.get_mut_or_fail() }
}

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
        unsafe {
            CAN_BUS_ONCE.init_once_with_function(|| {
                can2040::initialize_cbus(
                    &mut core,
                    CANBUS_FREQUENCY,
                    hpd_sys_rx!(pins).id().num as u32,
                    hpd_sys_tx!(pins).id().num as u32,
                )
            });
        }
        spawn_task(process_can_hpd_message());
    }

    start_global_executor();
    GLOBAL_LINEAR_BULL_STOP.store(false, Ordering::Relaxed);
    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

fn possibly_enqueue_hpd_message(f: CanFrame) {
    debug!("frame_to_hpd_msg() : convert can msg: {}", f);
    // 确认是HPD的消息。
    match f.id() {
        Id::Standard(sid) => {
            if sid.as_raw() != HPD_CAN_ID {
                // ignore HPD unrelated msgs.
                return;
            }
        }
        _ => {
            // Unsupport ids.
            return;
        }
    }
    match postcard::from_bytes::<HpdCommand>(&f.data()[..f.dlc()]) {
        Ok(hpd_msg) => get_hpd_mq().enqueue(hpd_msg),
        Err(err) => {
            error!("frame_to_hpd_msg(): errors in parsing HPD message, err: {}", Debug2Format(&err))
        }
    }
}

async fn process_can_hpd_message() {
    loop {
        Delay::new(1.millis()).await;

        match get_can().receive() {
            Ok(f) => {

                possibly_enqueue_hpd_message(f);
            }
            Err(nb::Error::Other(err)) => {
                error!("Canbus error: {}", err);
            }
            Err(_) => (),
        }
    }
}

// TODO(zephyr): Move the function to common/
fn send_can_message<U: Format + Serialize>(cob_id: u16, msg: U) -> Result<(), AtomiError> {
    info!("send_can_message(): to transmit msg: {}", msg);
    let data = postcard::to_allocvec::<U>(&msg).map_err(|_| AtomiError::UartInvalidInput)?;
    let f =
        CanFrame::new(StandardId::new(cob_id).ok_or(AtomiError::CanCobIdError)?, data.as_slice())
            .ok_or(AtomiError::CanFrameError)?;
    debug!("send_can_message(): to transmit frame: {}", f);
    match <Can2040 as embedded_can::blocking::Can>::transmit(get_can(), &f) {
        Ok(_) => {
            info!("Transmitted package: {:?}", f);
            Ok(())
        }
        Err(err) => {
            error!("Transmit error: {}", err);
            Err(AtomiError::CanTransmitError)?
        }
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
                HpdCommand::HpdPing => {
                    send_can_message(MC_CAN_ID, HpdCommand::HpdPong)
                }

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
                        // TODO(zephyr): 逻辑变了，MC端要一起改下，注释掉的代码如下：
                        // let _ = uart_comm.send(AtomiProto::AtomiError(AtomiError::HpdUnavailable));
                        // Err(AtomiError::HpdUnavailable)
                        send_can_message(MC_CAN_ID, HpdCommand::HpdBusy)
                    }
                }

                HpdCommand::HpdLinearBull(cmd) => {
                    if linear_bull_available {
                        let res = send_can_message(MC_CAN_ID, HpdCommand::HpdAck);
                        linear_bull_input_mq().enqueue(cmd);
                        linear_bull_available = false;
                        res
                    } else {
                        // TODO(zephyr): 逻辑变了，MC端要一起改下，注释掉的代码如下：
                        // let _ = uart_comm.send(AtomiProto::AtomiError(AtomiError::HpdUnavailable));
                        // Err(AtomiError::HpdUnavailable)
                        send_can_message(MC_CAN_ID, HpdCommand::HpdBusy)
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
