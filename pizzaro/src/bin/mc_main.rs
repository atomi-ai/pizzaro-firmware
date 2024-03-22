#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;

use cortex_m::peripheral::NVIC;
use defmt::{debug, error, info, Debug2Format};
use fugit::{ExtU64, RateExtU32};
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
use generic::atomi_proto::{wrap_result_into_proto, AtomiProto, McCommand};
use pizzaro::bsp::mc_ui_uart_irq;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, now, Delay};
use pizzaro::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::mc::system_executor::{
    process_executor_requests, system_executor_input_mq, system_executor_output_mq,
    wait_for_forward_dequeue, McSystemExecutor,
};
use pizzaro::mc::UiUartType;
use pizzaro::{common::async_initialization, mc_sys_rx, mc_sys_tx};
use pizzaro::{mc_485_dir, mc_uart, mc_ui_uart, mc_ui_uart_rx, mc_ui_uart_tx};

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

    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

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

        spawn_task(process_messages());
        spawn_task(process_executor_requests(McSystemExecutor::new(uart, Some(uart_dir))));
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

async fn process_messages() {
    // TODO(zephyr): Do we need to keep connection alive?
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
    // TODO(zephyr): Rethink: do we need to use AtomicBool for the lock?
    let mut system_locked = false;

    loop {
        Delay::new(1.millis()).await;

        if let Some(resp) = system_executor_output_mq().dequeue() {
            info!("[MC] get response from system executor: {}", resp);
            system_locked = false;
        }

        let t = get_mq().dequeue();
        // TODO(zephyr): simplify the code below.
        if t.is_some() {
            info!("[MC] get msg from queue: {}", t);
        }
        let msg = match t {
            None => continue, // no data, ignore
            Some(AtomiProto::Mc(McCommand::McPing)) => Ok(AtomiProto::Mc(McCommand::McPong)),
            Some(AtomiProto::Mc(McCommand::SystemRun(cmd))) => {
                if system_locked {
                    Err(AtomiError::McLockedForSystemRun)
                } else {
                    system_locked = true;
                    system_executor_input_mq().enqueue(AtomiProto::Mc(McCommand::SystemRun(cmd)));
                    Ok(AtomiProto::Mc(McCommand::McAck))
                }
            }
            Some(msg) => {
                if system_locked {
                    Err(AtomiError::McLockedForSystemRun)
                } else {
                    system_executor_input_mq().enqueue(msg);
                    wait_for_forward_dequeue()
                }
            }
        };
        info!("Processed result: {}", msg);

        match (|| -> Result<(), AtomiError> {
            let wrapped_msg = wrap_result_into_proto(msg);
            let binding =
                postcard::to_allocvec(&wrapped_msg).map_err(|_| AtomiError::DataConvertError)?;
            debug!("Data to send to PC: {}, wrapped_msg: {}", Debug2Format(&binding), wrapped_msg);
            let mut wr_ptr = binding.as_slice();
            while !wr_ptr.is_empty() {
                match serial.write(wr_ptr) {
                    Ok(len) => {
                        debug!("process_mc_message() 5.4, len = {}", len);
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
                debug!("Successfully send data back through USBCTRL");
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
                debug!("xfguo: got data: ({}) {}", count, &buf);
                match postcard::from_bytes::<AtomiProto>(&buf[..count]) {
                    Ok(message) => {
                        get_mq().enqueue(message);
                        debug!("{} | Received message: {:?}, added to mq", now().ticks(), message);
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
