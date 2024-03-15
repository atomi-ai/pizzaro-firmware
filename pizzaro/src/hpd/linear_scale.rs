use defmt::{Debug2Format, info};
use embedded_hal::digital::v2::InputPin;
use fugit::ExtU64;
use rp2040_hal::{pac, Sio};
use rp2040_hal::pac::{PIO0, RESETS};
use rp2040_hal::pio::{PIOExt, Rx, SM0};
use rp2040_hal::sio::SioFifo;
use rp_pico::hal;

use generic::atomi_error::AtomiError;

use crate::common::global_timer::Delay;
use crate::hpd::hpd_misc::LinearScale;

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

pub async fn read_linear_scale_with_pio(mut rx: Rx<(PIO0, SM0)>, linear_scale: &mut LinearScale) {
    loop {
        let mut encoder_measurement = 0;
        while let Some(encoder_value) = rx.read() {
            encoder_measurement = encoder_value as i32;
        }
        linear_scale.update_position(encoder_measurement);
        Delay::new(10.millis()).await;
    }
}

// TODO(zephyr): Use PIOExt / SM instead of PIO0.
pub fn init_quadrature_encoder<P: PIOExt>(encoder_pin0_id: u8, pio0: P, reset: &mut RESETS) -> Result<Rx<(P, SM0)>, AtomiError> {
    let encoder_program = pio_proc::pio_file!("src/hpd/quadrature_encoder.pio");
    info!("encoder_program = {:?}, encoder_pin0_id = {}", Debug2Format(&encoder_program.program), encoder_pin0_id);

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pio0.split(reset);
    let installed = pio.install(&encoder_program.program).unwrap();
    let (mut sm, rx, _) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .in_shift_direction(hal::pio::ShiftDirection::Left)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .jmp_pin(encoder_pin0_id)
        .in_pin_base(encoder_pin0_id)
        .clock_divisor_fixed_point(16, 0)
        .autopull(false)
        .autopush(false)
        .build(sm0);

    sm.set_pindirs([
        (encoder_pin0_id, hal::pio::PinDir::Input),
        (encoder_pin0_id + 1, hal::pio::PinDir::Input),
    ]);

    sm.start();
    Ok(rx)
}
