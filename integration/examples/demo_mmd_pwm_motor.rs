// 测试mmd的2只无刷电机和1只有刷电机

#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;

use cortex_m::asm::delay;
use defmt::info;
use generic::atomi_proto::{DispenserCommand, PeristalticPumpCommand};
use pizzaro::bsp::config::{MMD_BRUSHLESS_MOTOR_PWM_TOP, MMD_PERISTALTIC_PUMP_PWM_TOP};
use pizzaro::bsp::{MmdBrushlessMotor0Channel, MmdBrushlessMotor1Channel};
use pizzaro::common::brush_motor::BrushMotor;
use pizzaro::common::brushless_motor::BrushlessMotor;
use pizzaro::common::led_controller::{blinky_smart_led, MyLED};
use pizzaro::mmd::brush_motor_processor::MmdPeristalicPumpProcessor;
use pizzaro::mmd::brushless_motor_processor::DispenserMotorProcessor;
use pizzaro::{
    mmd_bl1_ctl_channel, mmd_bl1_ctl_pwm_slice, mmd_bl2_ctl_channel, mmd_bl2_ctl_pwm_slice,
    mmd_br0_pwm_slice, mmd_br_channel_a, mmd_br_channel_b, mmd_br_nEN, mmd_br_pwm_a, mmd_br_pwm_b,
    mmd_dir_bl0, mmd_dir_bl1, mmd_spd_ctrl_bl0, mmd_spd_ctrl_bl1, smart_led,
};
use rp2040_hal::Clock;
use rp2040_hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog, Timer};
use rp_pico::{entry, XOSC_CRYSTAL_FREQ};

use fugit::ExtU64;
use pizzaro::common::async_initialization;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, Delay};
use pizzaro::common::rp2040_timer::Rp2040Timer;
use rp2040_hal::pio::PIOExt;
use ws2812_pio::Ws2812Direct;

static mut BRUSHLESS_MOTOR_PROCESSOR: Option<DispenserMotorProcessor> = None;
static mut BRUSH_MOTOR_PROCESSOR: Option<MmdPeristalicPumpProcessor> = None;

#[entry]
fn main() -> ! {
    async_initialization();
    let mut pac = pac::Peripherals::take().unwrap();
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
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

    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    {
        // let mut led = pins.gpio2.into_push_pull_output().into_dyn_pin();
        // spawn_task(blinky(led));

        let my_led = MyLED::new(
            (1, 1, 1).into(),
            (0, 0, 0).into(),
            Ws2812Direct::new(
                smart_led!(pins).into_function().into_dyn_pin(),
                &mut pio,
                sm0,
                clocks.peripheral_clock.freq(),
            ),
        );

        spawn_task(blinky_smart_led(my_led));
    }

    let pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    {
        let mut pwm0 = mmd_bl1_ctl_pwm_slice!(pwm_slices);
        pwm0.set_ph_correct();
        pwm0.set_top(MMD_BRUSHLESS_MOTOR_PWM_TOP);
        pwm0.enable();
        mmd_bl1_ctl_channel!(pwm0).output_to(mmd_spd_ctrl_bl0!(pins));

        let mut pwm1 = mmd_bl2_ctl_pwm_slice!(pwm_slices);
        pwm1.set_ph_correct();
        pwm1.set_top(MMD_BRUSHLESS_MOTOR_PWM_TOP);
        pwm1.enable();
        mmd_bl2_ctl_channel!(pwm1).output_to(mmd_spd_ctrl_bl1!(pins));

        let dispenser0_motor = BrushlessMotor::new(
            mmd_dir_bl0!(pins).into_push_pull_output().into_dyn_pin(),
            pwm0,
            MmdBrushlessMotor0Channel,
            (0.03, 0.45, 0.55, 0.97),
            false,
            MMD_BRUSHLESS_MOTOR_PWM_TOP,
        );
        let dispenser1_motor = BrushlessMotor::new(
            mmd_dir_bl1!(pins).into_push_pull_output().into_dyn_pin(),
            pwm1,
            MmdBrushlessMotor1Channel,
            (0.03, 0.45, 0.55, 0.97),
            false,
            MMD_BRUSHLESS_MOTOR_PWM_TOP,
        );
        let brushless_motor_processor =
            DispenserMotorProcessor::new(dispenser0_motor, dispenser1_motor);

        unsafe {
            BRUSHLESS_MOTOR_PROCESSOR = Some(brushless_motor_processor);
        }
    }

    {
        let mut pwm = mmd_br0_pwm_slice!(pwm_slices);
        pwm.set_ph_correct();
        pwm.set_top(MMD_PERISTALTIC_PUMP_PWM_TOP);
        pwm.enable();
        mmd_br_channel_a!(pwm).output_to(mmd_br_pwm_a!(pins));
        mmd_br_channel_b!(pwm).output_to(mmd_br_pwm_b!(pins));
        pwm.channel_b.set_inverted();

        let peristaltic_pump_motor = BrushMotor::new(
            Some(mmd_br_nEN!(pins).into_push_pull_output().into_dyn_pin()),
            pwm,
            (0.03, 0.45, 0.55, 0.97),
            false,
            true,
            MMD_PERISTALTIC_PUMP_PWM_TOP,
        );
        let brush_motor_processor = MmdPeristalicPumpProcessor::new(peristaltic_pump_motor);
        unsafe {
            BRUSH_MOTOR_PROCESSOR = Some(brush_motor_processor);
        }
    }

    spawn_task(process_pwm_motor());
    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

pub async fn process_pwm_motor() {
    info!("process pwm motor tests");
    let brushless_motor_processor = unsafe { BRUSHLESS_MOTOR_PROCESSOR.as_mut().unwrap() };
    let brush_motor_processor = unsafe { BRUSH_MOTOR_PROCESSOR.as_mut().unwrap() };

    loop {
        brush_motor_processor.process(PeristalticPumpCommand::SetRotation { speed: 1000 });
        brushless_motor_processor.process(DispenserCommand::SetRotation {
            idx: 0,
            speed: 1000,
        });
        brushless_motor_processor.process(DispenserCommand::SetRotation {
            idx: 1,
            speed: 1000,
        });

        Delay::new(1.secs()).await;

        brush_motor_processor.process(PeristalticPumpCommand::SetRotation { speed: 0 });
        brushless_motor_processor.process(DispenserCommand::SetRotation { idx: 0, speed: 0 });
        brushless_motor_processor.process(DispenserCommand::SetRotation { idx: 1, speed: 0 });

        Delay::new(1.secs()).await;

        brush_motor_processor.process(PeristalticPumpCommand::SetRotation { speed: -500 });
        brushless_motor_processor
            .process(DispenserCommand::SetRotation {
                idx: 0,
                speed: -1000,
            })
            .unwrap();
        brushless_motor_processor
            .process(DispenserCommand::SetRotation {
                idx: 1,
                speed: -1000,
            })
            .unwrap();

        Delay::new(1.secs()).await;
    }
}
