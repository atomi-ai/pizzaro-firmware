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
use generic::atomi_proto::{wrap_result_into_proto, AtomiAutorun, AtomiProto, DispenserCommand, HpdCommand, LinearBullCommand, LinearStepperCommand, McCommand, MmdCommand, PeristalticPumpCommand, RotationStepperCommand, McSystemExecutorCmd};
use pizzaro::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, now, Delay};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::common::uart_comm::UartComm;
use pizzaro::{common::async_initialization, mc_sys_rx, mc_sys_tx};
use pizzaro::mc::system_executor::{forward, process_mc_one_full_run, system_executor_input_mq};
use pizzaro::mc::{UartDirType, UartType, UiUartType};

struct GlobalContainer {
    // 这个用法有点危险，虽然它可以工作。正常Uart是不应该作为一个共享变量被多个async future占用的，
    // 但是这里system_run占用的时候，是会用system_locked锁住的，所以应该还好。
    // 以后可以想想还有什么更好的办法，就是多个futures里面都需要有这个uart的话，而且我们也不希望
    // 提供thread-safe的uart的情况下，用什么模式来解决这个问题。
    uart: Option<UartType>,
    uart_dir: Option<UartDirType>,
    uart_comm: Option<UartComm<'static, UartDirType, UartType>>,
}
static mut GLOBAL_CONTAINER: GlobalContainer = GlobalContainer {
    uart: None,
    uart_dir: None,
    uart_comm: None
};

// TODO(lv): Put all static variables into GlobalContainer.
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;
static mut UIUART: Option<UiUartType> = None;

static mut FROM_PC_MESSAGE_QUEUE: Once<MessageQueueWrapper<AtomiProto>> = Once::new();
fn get_mq() -> &'static mut MessageQueueWrapper<AtomiProto> {
    unsafe { FROM_PC_MESSAGE_QUEUE.get_mut() }
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

        unsafe {
            GLOBAL_CONTAINER.uart.replace(uart);
            GLOBAL_CONTAINER.uart_dir.replace(uart_dir);
            let mut uart_comm = UartComm::new(
                GLOBAL_CONTAINER.uart.as_mut().unwrap(),
                &mut GLOBAL_CONTAINER.uart_dir,
                UART_EXPECTED_RESPONSE_LENGTH,
            );
            GLOBAL_CONTAINER.uart_comm.replace(uart_comm);
        }
        let uart_comm_rc0 = unsafe { GLOBAL_CONTAINER.uart_comm.as_mut().unwrap() };
        let uart_comm_rc1 = unsafe { GLOBAL_CONTAINER.uart_comm.as_mut().unwrap() };

        spawn_task(process_messages(uart_comm_rc0));
        spawn_task(process_mc_one_full_run(uart_comm_rc1));
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

    // spawn_task(dump_executor_status());
    start_global_executor();

    loop {
        // info!("rs485_send: in loop (should not be here)");
    }
}

async fn process_messages(uart_comm: &mut UartComm<'_, UartDirType, UartType>) {
    // TODO(zephyr): Do we need to keep connection alive?
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
    let mut system_locked = false;

    loop {
        Delay::new(1.millis()).await;

        let t = get_mq().dequeue();
        // TODO(zephyr): simplify the code below.
        if t.is_some() {
            info!("[MC] get msg from queue: {}", t);
        }
        let msg = match t {
            None => continue, // no data, ignore
            Some(AtomiProto::Mc(McCommand::McPing)) => {
                Ok(AtomiProto::Mc(McCommand::McPong))
            }
            Some(AtomiProto::Mc(McCommand::FullRun)) => {
                if system_locked {
                    Err(AtomiError::McLockedForSystemRun)
                } else {
                    system_locked = true;
                    system_executor_input_mq().enqueue(McSystemExecutorCmd::ExecuteOneFullRun);
                    Ok(AtomiProto::Mc(McCommand::McAck))
                }
            }
            Some(msg) => {
                if system_locked {
                    Err(AtomiError::McLockedForSystemRun)
                } else {
                    forward(uart_comm, msg).await
                }
            },
        };
        info!("Processed result: {}", msg);

        match (|| -> Result<(), AtomiError> {
            let wrapped_msg = wrap_result_into_proto(msg);
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
