#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;

use cortex_m::peripheral::NVIC;
use defmt::{error, info, warn, Debug2Format};
use fugit::{ExtU64, RateExtU32};
use pizzaro::bsp::{mc_ui_uart_irq, McUartDirPinType, McUartType, McUiUartType};
use pizzaro::{mc_485_dir, mc_uart, mc_ui_uart, mc_ui_uart_rx, mc_ui_uart_tx};
use rp2040_hal::gpio::FunctionUart;
use rp2040_hal::uart::{DataBits, StopBits, UartConfig};
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    uart::UartPeripheral,
    watchdog::Watchdog,
    Timer,
};
use rp_pico::hal::pac::interrupt;
use rp_pico::{entry, hal, XOSC_CRYSTAL_FREQ};
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usb_device::UsbError;
use usbd_serial::SerialPort;

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{
    wrap_result_into_proto, AtomiAutorun, AtomiProto, DispenserCommand, HpdCommand,
    LinearBullCommand, LinearStepperCommand, McCommand, MmdCommand, PeristalticPumpCommand,
    RotationStepperCommand,
};
use pizzaro::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, now, Delay};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::uart_comm::UartComm;
use pizzaro::{common::async_initialization, mc_sys_rx, mc_sys_tx};

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;
static mut UIUART: Option<UiUartType> = None;
static mut FROM_PC_MESSAGE_QUEUE: Once<MessageQueueWrapper<AtomiProto>> = Once::new();
static mut AUTORUN_MESSAGE_QUEUE: Once<MessageQueueWrapper<AtomiAutorun>> = Once::new();
static mut AUTORUN_RESPONSE_QUEUE: Once<MessageQueueWrapper<AtomiProto>> = Once::new();
fn get_mq() -> &'static mut MessageQueueWrapper<AtomiProto> {
    unsafe { FROM_PC_MESSAGE_QUEUE.get_mut() }
}

fn get_autorun_mq() -> &'static mut MessageQueueWrapper<AtomiAutorun> {
    unsafe { AUTORUN_MESSAGE_QUEUE.get_mut() }
}

fn get_autorun_resp_mq() -> &'static mut MessageQueueWrapper<AtomiProto> {
    unsafe { AUTORUN_RESPONSE_QUEUE.get_mut() }
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

    {
        // Initialize UART
        let uart_pins = (
            // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
            mc_sys_tx!(pins).into_function::<FunctionUart>(),
            // UART RX (characters received by RP2040) on pin 2 (GPIO1)
            mc_sys_rx!(pins).into_function::<FunctionUart>(),
        );
        let uart = UartPeripheral::new(mc_uart!(pac), uart_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();
        let uart_dir = mc_485_dir!(pins).reconfigure();

        spawn_task(process_messages(uart, Some(uart_dir)));
    }

    {
        // initialize UI screen
        let ui_uart_pins = (
            // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
            mc_ui_uart_tx!(pins).into_function::<FunctionUart>(),
            // UART RX (characters received by RP2040) on pin 2 (GPIO1)
            mc_ui_uart_rx!(pins).into_function::<FunctionUart>(),
        );

        let ui_uart = UartPeripheral::new(mc_ui_uart!(pac), ui_uart_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        unsafe {
            UIUART = Some(ui_uart);
            NVIC::unmask(mc_ui_uart_irq());
        }

        //spawn_task(process_ui_screen(ui_uart));
    }

    {
        // Initialize USBCTRL (uart-over-usb)

        // Set up the USB driver
        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        ));
        unsafe {
            // Note (safety): This is safe as interrupts haven't been started yet
            USB_BUS = Some(usb_bus);
        }

        // Grab a reference to the USB Bus allocator. We are promising to the
        // compiler not to take mutable access to this global variable whilst this
        // reference exists!
        let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

        // Set up the USB Communications Class Device driver
        let serial = SerialPort::new(bus_ref);
        unsafe {
            USB_SERIAL = Some(serial);
        }

        // Create a USB device with a fake VID and PID
        let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(2) // from: https://www.usb.org/defined-class-codes
            .build();
        unsafe {
            // Note (safety): This is safe as interrupts haven't been started yet
            USB_DEVICE = Some(usb_dev);
        }

        // Enable the USB interrupt
        unsafe {
            pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        };
    }

    spawn_task(autorun_logic());
    // spawn_task(dump_executor_status());
    start_global_executor();

    loop {
        // info!("rs485_send: in loop (should not be here)");
    }
}

type UartType = McUartType;
type UartDirType = McUartDirPinType;
type UiUartType = McUiUartType;

async fn process_messages(mut uart: UartType, mut uart_dir: Option<UartDirType>) {
    // TODO(zephyr): Do we need to keep connection alive?
    let mut uart_comm = UartComm::new(&mut uart, &mut uart_dir, UART_EXPECTED_RESPONSE_LENGTH);
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };

    loop {
        Delay::new(1.millis()).await;

        let t = get_mq().dequeue();
        // TODO(zephyr): simplify the code below.
        if t.is_some() {
            info!("[MC] get msg from queue: {}", t);
        }
        let msg = match t {
            None => continue, // no data, ignore
            // 处理autorun内部的延迟需求（这个有必要么？）
            Some(AtomiProto::Autorun(AtomiAutorun::Wait { seconds })) => {
                let _ = Delay::new((seconds as u64).secs()).await;
                Ok(AtomiProto::Autorun(AtomiAutorun::Done))
            }
            Some(AtomiProto::Mc(mc_cmd)) => process_mc_message(mc_cmd),
            // 所有autorun请求丢给autorun的task
            Some(AtomiProto::Autorun(cmd)) => {
                get_autorun_mq().enqueue(cmd);
                Ok(AtomiProto::Autorun(AtomiAutorun::Done))
            }
            Some(msg) => forward(&mut uart_comm, msg).await,
        };
        info!("Processed result: {}", msg);

        match (|| -> Result<(), AtomiError> {
            let wrapped_msg = wrap_result_into_proto(msg);
            get_autorun_resp_mq().enqueue(wrapped_msg);
            let binding =
                postcard::to_allocvec(&wrapped_msg).map_err(|_| AtomiError::DataConvertError)?;
            info!(
                "Data to send to PC: {}, wrapped_msg: {}",
                Debug2Format(&binding),
                wrapped_msg
            );
            let mut wr_ptr = binding.as_slice();
            while !wr_ptr.is_empty() {
                match serial.write(wr_ptr) {
                    Ok(len) => {
                        info!("process_mc_message() 5.4, len = {}", len);
                        if len > wr_ptr.len() {
                            error!("process_messages() 5.5: overflow happens");
                            return Err(AtomiError::UsbCtrlWriteError);
                        }
                        wr_ptr = &wr_ptr[len..]
                    }
                    Err(_) => Err(AtomiError::UsbCtrlWriteError)?,
                };
            }
            Ok(())
        })() {
            Ok(_) => {
                info!("Successfully send data back through USBCTRL");
            }
            Err(err) => {
                error!("Errors in sending data back through USBCTRL, {}", err)
            }
        }
    }
}

async fn forward(
    uart_comm: &mut UartComm<'_, UartDirType, UartType>,
    msg: AtomiProto,
) -> Result<AtomiProto, AtomiError> {
    info!("Forward msg: {}", msg);
    if let Err(e) = uart_comm.send(msg) {
        error!("Error in sending command, err: {:?}", e);
        return Err(AtomiError::UartWriteError);
    }

    // 异步读取响应长度 or timeout
    uart_comm.recv_timeout::<AtomiProto>(500.millis()).await
}

fn process_mc_message(msg: McCommand) -> Result<AtomiProto, AtomiError> {
    info!("Process MC message: {}", &msg);
    match msg {
        McCommand::McPing => Ok(AtomiProto::Mc(McCommand::McPong)),
        _ => Err(AtomiError::UnsupportMcCommand),
    }
}

#[interrupt]
unsafe fn USBCTRL_IRQ() {
    use core::sync::atomic::{AtomicBool, Ordering};

    /// Note whether we've already printed the "hello" message.
    static SAID_HELLO: AtomicBool = AtomicBool::new(false);

    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    // Say hello exactly once on start-up
    if !SAID_HELLO.load(Ordering::Relaxed) {
        SAID_HELLO.store(true, Ordering::Relaxed);
        let _ = serial.write(b"Hello, World!\r\n");
    }

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Err(UsbError::WouldBlock) => {} // ignore
            Err(err) => {
                info!("xfguo: got error: {:?}", Debug2Format(&err));
            }
            Ok(0) => {
                info!("xfguo: ok(0)");
            }
            Ok(count) => {
                info!("xfguo: got data: ({}) {}", count, &buf);
                match postcard::from_bytes::<AtomiProto>(&buf[..count]) {
                    Ok(message) => {
                        get_mq().enqueue(message);
                        info!(
                            "{} | Received message: {:?}, added to mq",
                            now().ticks(),
                            message
                        );
                    }
                    Err(err) => info!(
                        "Failed to parse message: {}, err: {}",
                        Debug2Format(&buf[..count]),
                        Debug2Format(&err)
                    ),
                }
            }
        }
    }
}

fn check_autorun_status() -> Option<AtomiAutorun> {
    match get_autorun_mq().dequeue() {
        None => None,
        Some(auto_run) => Some(auto_run),
    }
}

//type ScriptType = (AtomiProto, i32);
type ScriptType = AtomiProto;

async fn autorun_logic() {
    let mut engage = false;
    const SCRIPT: &[ScriptType] = &[
        // 流程启动
        // 伸缩传送带归位
        AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::MoveTo {
            position: 0,
        })),
        // 压饼
        // AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::MoveTo {
        //     position: 53200,
        // })),
        // 等待执行到位
        // AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::WaitIdle)),

        // 收回
        AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::MoveTo {
            position: 0,
        })),
        // 等待执行到位
        // AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::WaitIdle)),

        // 启动蠕动泵
        // AtomiProto::Mmd(MmdCommand::MmdPeristalticPump(
        //     PeristalticPumpCommand::SetRotation { speed: 300 },
        // )),
        // 启动按压平台旋转
        AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetPresserRotation { speed: 200 },
        )),
        // 伸缩传送带伸出，此时均匀铺番茄酱
        AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::MoveTo {
            position: 200,
        })),
        // 等待执行到位
        AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::WaitIdle)),
        // 停止蠕动泵
        // AtomiProto::Mmd(MmdCommand::MmdPeristalticPump(
        //     PeristalticPumpCommand::SetRotation { speed: 0 },
        // )),
        // 传送带旋转
        AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetConveyorBeltRotation { speed: 200 },
        )),
        // 启动起司料斗
        AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
            idx: 0,
            speed: 1000,
        })),
        // 伸缩带退回
        AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::MoveTo {
            position: 0,
        })),
        // 等待执行到位
        AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::WaitIdle)),
        // 传送带停止
        AtomiProto::Mmd(MmdCommand::MmdRotationStepper(
            RotationStepperCommand::SetConveyorBeltRotation { speed: 0 },
        )),
        // 起司料斗停止
        AtomiProto::Mmd(MmdCommand::MmdDisperser(DispenserCommand::SetRotation {
            idx: 0,
            speed: 0,
        })),
        // 流程结束
    ];
    loop {
        Delay::new(1.millis()).await;

        for step in SCRIPT {
            // waiting for engaging
            while !engage {
                Delay::new(1.millis()).await;
                match check_autorun_status() {
                    Some(AtomiAutorun::Start) => engage = true,
                    Some(AtomiAutorun::Stop) => engage = false,
                    _ => (),
                }
            }

            warn!("auto run:{}", step);
            // 如果step为wait_idle，需要一直等到成功为止
            // 但在那里收返回的应答呢？mc会把所有的应答直接一股脑的forward到usb，
            match step.clone() {
                AtomiProto::Hpd(HpdCommand::HpdLinearBull(LinearBullCommand::WaitIdle)) => {
                    loop {
                        get_autorun_mq().clear();
                        get_mq().enqueue(step.clone());

                        let resp = get_autorun_resp_mq().dequeue();
                        if let Some(resp) = resp {
                            if resp == AtomiProto::Hpd(HpdCommand::HpdAck) {
                                //got idle signal
                                break;
                            }
                        }
                        Delay::new(100.millis()).await;
                    }
                }
                AtomiProto::Mmd(MmdCommand::MmdLinearStepper(LinearStepperCommand::WaitIdle)) => {
                    loop {
                        get_mq().enqueue(step.clone());
                        let resp = get_autorun_resp_mq().dequeue();
                        if let Some(resp) = resp {
                            if resp == AtomiProto::Mmd(MmdCommand::MmdAck) {
                                //got idle signal
                                break;
                            }
                        }
                        Delay::new(100.millis()).await;
                    }
                }
                _ => (),
            }
            get_mq().enqueue(step.clone());
        }
        engage = false;
    }
}
