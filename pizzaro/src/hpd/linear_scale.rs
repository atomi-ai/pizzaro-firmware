use crate::common::global_timer::Delay;
use crate::hpd::hpd_misc::LinearScale;
use embedded_hal::digital::v2::InputPin;
use fugit::ExtU64;
use rp2040_hal::sio::SioFifo;
use rp2040_hal::{pac, Sio};
use rp_pico::hal;

pub fn core1_task() -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };

    let mut sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let gpio10 = pins.gpio10.into_floating_input();
    let gpio11 = pins.gpio11.into_floating_input();
    let (mut x, mut y) = (true, false);
    let mut pos = 0i32;
    let mut count = 0;
    loop {
        let new_x = gpio10.is_high().unwrap_or(false);
        let new_y = gpio11.is_high().unwrap_or(true);
        match (x, y, new_x, new_y) {
            (false, false, true, false)
            | (false, true, false, false)
            | (true, false, true, true)
            | (true, true, false, true) => {
                // increment
                pos += 1;
            }
            (false, false, false, true)
            | (false, true, true, true)
            | (true, false, false, false)
            | (true, true, true, false) => {
                // decrement
                pos -= 1;
            }
            (_, _, _, _) => {}
        }
        (x, y) = (new_x, new_y);
        count += 1;
        if count % 500 == 13 {
            sio.fifo.write(pos as u32);
        }
    }
}

pub async fn read_and_update_linear_scale(mut fifo: SioFifo, linear_scale: &mut LinearScale) {
    loop {
        {
            let mut last_pos: Option<i32> = None;
            // let mut count = 0;
            while let Some(pos) = fifo.read() {
                // count += 1;
                last_pos = Some(pos as i32);
            }
            if let Some(t) = last_pos {
                // info!("count = {}, pos = {}", count, t);
                linear_scale.update_position(t);
            }
        }
        Delay::new(1.millis()).await;
    }
}
