// 测试mmd的42步进和57步进的旋转模式
// 注意，只测试第二只42步进，第一只42步进是直线运动，由demo_linear_actuator测试

#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;

use cortex_m::asm::delay;
use defmt::info;
use pizzaro::common::led_controller::{blinky_smart_led, MyLED};
use pizzaro::mmd::rotation_stepper_processor::RotationStepperProcessor;
use pizzaro::smart_led;
use pizzaro::{
    bsp::{MmdMotor42Step1Channel, MmdMotor57StepChannel},
    mmd_motor42_pwm_slice1, mmd_motor42_step1_channel, mmd_stepper42_dir1, mmd_stepper42_nEN1,
    mmd_stepper42_step1, mmd_stepper57_dir, mmd_stepper57_nEN, mmd_stepper57_pwm_slice,
    mmd_stepper57_step, mmd_stepper57_step_channel,
};
use rp2040_hal::Clock;
use rp2040_hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog, Timer};
use rp_pico::{entry, XOSC_CRYSTAL_FREQ};

use fugit::ExtU64;
use pizzaro::common::async_initialization;
use pizzaro::common::executor::{spawn_task, start_global_executor};
use pizzaro::common::global_timer::{init_global_timer, Delay};
use pizzaro::common::rp2040_timer::Rp2040Timer;
use pizzaro::mmd::pwm_stepper::PwmStepper;
use rp2040_hal::pio::PIOExt;
use ws2812_pio::Ws2812Direct;

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

    let pins =
        rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

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
        // 使用第二个通道连接驱动传送带旋转的电机
        let enable_pin_42 = mmd_stepper42_nEN1!(pins);
        let dir_pin_42 = mmd_stepper42_dir1!(pins).into_push_pull_output().into_dyn_pin();

        let mut pwm_42 = mmd_motor42_pwm_slice1!(pwm_slices);
        mmd_motor42_step1_channel!(pwm_42).output_to(mmd_stepper42_step1!(pins));

        // init 57
        let enable_pin_57 = Some(mmd_stepper57_nEN!(pins).into_push_pull_output().into_dyn_pin());
        let dir_pin_57 = mmd_stepper57_dir!(pins).into_push_pull_output().into_dyn_pin();
        //let step_pin_57 = mmd_stepper57_step!(pins).into_push_pull_output();

        let mut pwm_57 = mmd_stepper57_pwm_slice!(pwm_slices);
        mmd_stepper57_step_channel!(pwm_57).output_to(mmd_stepper57_step!(pins));

        let mut processor = RotationStepperProcessor::new(
            PwmStepper::new(
                enable_pin_42,
                dir_pin_42,
                MmdMotor42Step1Channel,
                clocks.peripheral_clock.freq(),
                pwm_42,
                200, // 无细分，一圈200脉冲
                false,
            ),
            PwmStepper::new(
                enable_pin_57,
                dir_pin_57,
                MmdMotor57StepChannel,
                clocks.peripheral_clock.freq(),
                pwm_57,
                200, // 无细分，一圈200脉冲
                false,
            ),
        );
        processor.enable();
        spawn_task(process_rotation_stepper(processor));
    }

    start_global_executor();

    loop {
        info!("in loop");
        delay(120_000_000);
    }
}

pub async fn process_rotation_stepper(mut processor: RotationStepperProcessor) {
    info!("process_mmd_rotation_stepper_message() 0");
    loop {
        // processor
        //     .process_rotation_stepper_request(
        //         generic::atomi_proto::RotationStepperCommand::SetConveyorBeltRotation {
        //             speed: 200,
        //         },
        //     )
        //     .await;
        processor
            .process_rotation_stepper_request(
                generic::atomi_proto::RotationStepperCommand::SetPresserRotation { speed: 1000 },
            )
            .await;

        Delay::new(1.secs()).await;

        processor
            .process_rotation_stepper_request(
                generic::atomi_proto::RotationStepperCommand::SetConveyorBeltRotation { speed: 0 },
            )
            .await;
        processor
            .process_rotation_stepper_request(
                generic::atomi_proto::RotationStepperCommand::SetPresserRotation { speed: 0 },
            )
            .await;

        Delay::new(1.secs()).await;
        // processor
        //     .process_rotation_stepper_request(
        //         generic::atomi_proto::RotationStepperCommand::SetConveyorBeltRotation {
        //             speed: -100,
        //         },
        //     )
        //     .await;
        processor
            .process_rotation_stepper_request(
                generic::atomi_proto::RotationStepperCommand::SetPresserRotation { speed: -500 },
            )
            .await;

        Delay::new(1.secs()).await;
    }
}
