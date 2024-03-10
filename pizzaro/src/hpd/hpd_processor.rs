use defmt::{info, Debug2Format};
use fugit::ExtU64;
use rp2040_hal::gpio::bank0::{Gpio8, Gpio9};
use rp2040_hal::gpio::{FunctionUart, Pin, PullDown};
use rp2040_hal::pac::UART1;
use rp2040_hal::uart::{Enabled, UartPeripheral};

use generic::atomi_error::AtomiError;
use generic::atomi_proto::{AtomiProto, HpdCommand};

use crate::bsp::config::HPD_PID;
use crate::common::global_status::{set_status, FutureStatus, FutureType};
use crate::common::global_timer::{now, Delay};
use crate::common::uart_comm::UartComm;
use crate::hpd::hpd_misc::{HpdDirection, LinearScale, PwmMotor};
use crate::hpd::pid::PIDController;

pub type HpdUartType = UartPeripheral<
    Enabled,
    UART1,
    (
        Pin<Gpio8, FunctionUart, PullDown>,
        Pin<Gpio9, FunctionUart, PullDown>,
    ),
>;

pub struct HpdProcessor {
    linear_scale: &'static mut LinearScale,
    pwm_motor: PwmMotor,
}

impl HpdProcessor {
    pub fn new(linear_scale: &'static mut LinearScale, pwm_motor: PwmMotor) -> Self {
        Self {
            linear_scale,
            pwm_motor,
        }
    }

    /*
       TODO(zephyr): 感觉收发消息一个future，move应该还需要一个future。
           要不收发消息的future就会被block，整个系统感觉就不对了。
    */
    pub async fn process_hpd_message<'a>(
        &mut self,
        uart_comm: &mut UartComm<'a, HpdUartType>,
        msg: HpdCommand,
    ) -> Result<(), AtomiError> {
        set_status(FutureType::Hpd, FutureStatus::HpdBusy);
        match msg {
            HpdCommand::HpdPing => uart_comm.send(AtomiProto::Hpd(HpdCommand::HpdPong))?,
            HpdCommand::HpdHome => {
                uart_comm.send(AtomiProto::Hpd(HpdCommand::HpdBusy))?;
                self.home().await.map(|_| ())?
            }
            HpdCommand::HpdMoveTo { position } => {
                uart_comm.send(AtomiProto::Hpd(HpdCommand::HpdBusy))?;
                self.move_to(position).await.map(|_| ())?
            }
            HpdCommand::HpdMoveToRelative { distance } => {
                uart_comm.send(AtomiProto::Hpd(HpdCommand::HpdBusy))?;
                self.move_to_relative(distance).await.map(|_| ())?
            }
            _ => Err(AtomiError::UnaccepableCommand)?,
        }
        set_status(FutureType::Hpd, FutureStatus::HpdAvailable);
        Ok(())
    }

    fn check_homed(&self) -> Result<i32, AtomiError> {
        self.linear_scale.check_homed()
    }

    pub async fn home(&mut self) -> Result<i32, AtomiError> {
        self.pwm_motor.start_pwm_motor()?;

        let dir = HpdDirection::Bottom;
        info!(
            "xfguo: Home to one direction, dir: {}, from: {}, to: {}",
            dir,
            self.linear_scale.get_rel_position(),
            dir.get_most_position()
        );
        self.home_on_direction(dir).await?;
        // info!(
        //     "xfguo: Home before set_top_position, current = {}",
        //     Debug2Format(self.linear_scale)
        // );
        // self.linear_scale.set_top_position();
        // dir = dir.reverse();
        // info!(
        //     "xfguo: Home to another direction, current = {}, dir: {}, target: {},",
        //     Debug2Format(self.linear_scale),
        //     dir,
        //     dir.get_most_position()
        // );
        // self.home_on_direction(dir).await?;
        info!(
            "xfguo: After homed, linear_scale = {}",
            Debug2Format(self.linear_scale)
        );
        self.linear_scale.set_home();
        info!(
            "xfguo: After homed 2, linear_scale = {}",
            Debug2Format(self.linear_scale)
        );

        self.linear_scale.get_abs_position()
    }

    async fn home_on_direction(&mut self, dir: HpdDirection) -> Result<i32, AtomiError> {
        info!("home_on_direction 0, now = {}", now().ticks());
        self.move_with_speed(dir.get_dir_seg() * 1.0, 10_000_000)
            .await?;
        info!("home_on_direction 2, now = {}", now().ticks());
        self.move_with_speed(dir.get_dir_seg() * -1.0, 1000).await?;
        info!("home_on_direction 4, now = {}", now().ticks());
        let t = self
            .move_with_speed(dir.get_dir_seg() * 1.0, 10_000_000)
            .await;
        info!("home_on_direction 9, now = {}", now().ticks());
        t
    }

    async fn move_with_speed(&mut self, speed: f32, repeat_times: i32) -> Result<i32, AtomiError> {
        info!("xfguo: speed = {}, repeat_times = {}", speed, repeat_times);
        self.linear_scale.reset_stationary();
        for _ in 0..repeat_times {
            Delay::new(1.millis()).await;
            if self.linear_scale.is_stationary() {
                break;
            }
            //info!("pos = {}", self.linear_scale.get_rel_position());
            self.pwm_motor.apply_speed(speed);
        }
        self.pwm_motor.apply_speed(0.0);
        self.linear_scale.get_rel_position()
    }

    /// Arguments:
    ///   - distance: unit in 0.01mm
    pub async fn move_to_relative(&mut self, distance: i32) -> Result<i32, AtomiError> {
        let target = self.linear_scale.get_rel_position()? + distance;
        info!("xfguo: move_to_relative() 1, target: {}", target);
        let mut pid = PIDController::new(HPD_PID.0, HPD_PID.1, HPD_PID.2, target);
        let dt = 0.01;

        loop {
            Delay::new(1.millis()).await;
            let pos = self.linear_scale.get_rel_position()?;
            if pid.reach_target(pos, now()) {
                break;
            }
            // if self.linear_scale.is_stationary() {
            //     break;
            // }
            // Only trigger PID when the position is changed.
            let speed = pid.calculate(pos, dt);
            info!(
                "Current pos: {}, speed = {}, target = {}",
                pos, speed, target
            );
            self.pwm_motor.apply_speed(speed);
        }
        self.pwm_motor.apply_speed(0.0);
        self.linear_scale.get_rel_position()
    }

    pub async fn move_to(&mut self, target_position: i32) -> Result<i32, AtomiError> {
        self.check_homed()?;
        self.move_to_relative(self.linear_scale.get_distance(target_position)?)
            .await
    }
}
