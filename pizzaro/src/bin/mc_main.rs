#![no_std]
#![no_main]

extern crate alloc;

use panic_probe as _;
use defmt_rtt as _;

use alloc::boxed::Box;
use defmt::{Debug2Format, error, info};
use fugit::{ExtU64, RateExtU32};
use rp2040_hal::{clocks::{init_clocks_and_plls, Clock}, pac, sio::Sio, Timer, uart::{UartPeripheral}, watchdog::Watchdog};
use rp2040_hal::uart::{DataBits, Enabled, StopBits, UartConfig};
use rp_pico::{entry, hal, XOSC_CRYSTAL_FREQ};
use rp_pico::hal::pac::interrupt;
use rp2040_hal::gpio::{FunctionUart, Pin, PullDown};
use rp2040_hal::gpio::bank0::{Gpio12, Gpio13};
use rp_pico::pac::UART0;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usb_device::UsbError;
use usbd_serial::SerialPort;
use pizzaro::common::async_initialization;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{Delay, init_global_timer, now};
use pizzaro::common::rp2040_timer::Rp2040Timer;
use generic::atomi_error::AtomiError;
use generic::atomi_proto::{AtomiProto, McCommand};
use pizzaro::common::consts::UART_EXPECTED_RESPONSE_LENGTH;
use pizzaro::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use pizzaro::common::once::Once;
use pizzaro::common::uart_comm::UartComm;

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

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
            pins.gpio12.into_function::<FunctionUart>(),
            // UART RX (characters received by RP2040) on pin 2 (GPIO1)
            pins.gpio13.into_function::<FunctionUart>(),
        );
        let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();
        spawn_task(process_messages(uart));
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

type UartType = UartPeripheral<Enabled, UART0, (
    Pin<Gpio12, FunctionUart, PullDown>,
    Pin<Gpio13, FunctionUart, PullDown>)>;

async fn process_messages(mut uart: UartType) {
    // TODO(zephyr): Do we need to keep connection alive?
    let mut uart_comm = UartComm::new(&mut uart, UART_EXPECTED_RESPONSE_LENGTH);
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };

    loop {
        Delay::new(1.millis()).await;

        let t = get_mq().dequeue();
        // TODO(zephyr): simplify the code below.
        if !t.is_none() {
            info!("[MC] get msg from queue: {}", t);
        }
        let msg = match t {
            None => continue,  // no data, ignore
            Some(AtomiProto::Mc(mc_cmd)) => process_mc_message(mc_cmd),
            Some(msg) => {
                forward(&mut uart_comm, msg).await
            }
        };
        info!("Processed result: {}", msg);

        let res = (|| -> Result<(), AtomiError> {
            let binding = postcard::to_allocvec(&msg?).map_err(|_| AtomiError::DataConvertError)?;
            info!("Data to send to PC: {}", Debug2Format(&binding));
            let mut wr_ptr = binding.as_slice();
            while !wr_ptr.is_empty() {
                match serial.write(wr_ptr) {
                    Ok(len) => wr_ptr = &wr_ptr[len..],
                    Err(_) => Err(AtomiError::UsbCtrlWriteError)?,
                };
            }
            Ok(())
        })();
        info!("Send data back through USBCTRL result: {}", res);
    }
}

async fn forward(uart_comm: &mut UartComm<'_, UartType>, msg: AtomiProto) -> Result<AtomiProto, AtomiError> {
    info!("Forward msg: {}", msg);
    if let Err(e) = uart_comm.send(msg) {
        error!("Error in sending command, err: {:?}", e);
        return Err(AtomiError::UartWriteError);
    }

    // 异步读取响应长度
    uart_comm.recv::<AtomiProto>().await
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
            Err(UsbError::WouldBlock) => {},  // ignore
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
                        info!("{} | Received message: {:?}, added to mq", now().ticks(), message);
                    },
                    Err(err) => info!("Failed to parse message: {}, err: {}", Debug2Format(&buf[..count]), Debug2Format(&err)),
                }
            }
        }
    }
}
