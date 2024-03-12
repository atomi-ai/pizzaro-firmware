//! Declaration of the TMC2209 registers and their implementations.
//!
//! Please refer to the TMC2209 datasheet for information on what each of these registers and their
//! fields mean. The register map is described under section 5 of the datasheet.
//!
//! https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

#![allow(non_camel_case_types)]
use bitfield::bitfield;

// Register Traits
// --------------------------------------------------------

/// Implemented for all register types.
///
/// NOTE: This should not be implemented for custom types. If the user attempts to request a custom
/// register type from the register `Map`, the method call will hang indefinitely.
pub trait Register: Into<State> {
    const ADDRESS: Address;
}

/// Implemented for all registers that can be read from.
pub trait ReadableRegister: Register + From<u32> {}

/// Implemented for all registers that can be written to.
pub trait WritableRegister: Register + Into<u32> {}

/// An error that might occur in the case that an address could not be parsed.
#[derive(Debug)]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct UnknownAddress;

/// An error indicating an unexpected `State`.
#[derive(Debug)]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct UnexpectedAddress;

// Register Declarations
// --------------------------------------------------------

bitfield! {
    #[derive(Clone, Copy, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct GCONF(u32);
    impl Debug;
    u32;
    pub recalibrate, set_recalibrate: 0;
    pub faststandstill, set_faststandstill: 1;
    pub en_pwm_mode, set_en_pwm_mode: 2;
    pub multistep_filt, set_multistep_filt: 3;
    pub shaft, set_shaft: 4;
    pub diag0_error, set_diag0_error: 5;
    pub diag0_otpw, set_diag0_otpw:6;
    pub diag0_stall, set_diag0_stall: 7; // SD_MODE=1
    pub diag0_step, set_diag0_step: 7;   // SD_MODE=0
    pub diag1_stall, set_diag1_stall: 8; // SD_MODE=1
    pub diag1_dir, set_diag1_dir: 8;   // SD_MODE=0
    pub diag1_index, set_diag1_index:9;
    pub diag1_onstate, set_diag1_onstate:10;
    pub diag1_steps_skipped, set_diag1_steps_skipped:11;
    pub diag0_int_pushpull, set_diag0_int_pushpull:12;
    pub diag1_poscomp_pushpull, set_diag1_poscomp_pushpull:13;
    pub small_hysteresis, set_small_hysteresis:14;
    pub stop_enable, set_stop_enable:15;
    pub direct_mode, set_direct_mode:16;
    pub test_mode, set_test_mode:17;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct GSTAT(u32);
    impl Debug;
    u8;
    pub reset, _: 0;
    pub drv_err, _: 1;
    pub uv_cp, _: 2;
}

#[derive(Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
#[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct IFCNT(pub u32);

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct SLAVECONF(u32);
    impl Debug;
    u8;
    pub slaveaddr, set_slaveaddr: 7, 0;
    pub senddelay, set_senddelay: 11, 8;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct OTP_PROG(u32);
    impl Debug;
    u16;
    pub otp_bit, set_otp_bit: 2, 0;
    pub otp_byte, set_otp_byte: 5, 4;
    pub opt_magic, set_otp_magic: 15, 8;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct OTP_READ(u32);
    impl Debug;
    u8;
    pub otp_fclktrim, _: 4, 0;
    pub otp_s2_level, _: 5;
    pub otp_bbm, _: 6;
    pub otp_tbl, _: 7;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct IOIN(u32);
    impl Debug;
    u8;
    pub refl_step, _: 0;
    pub refr_dir, _: 1;
    pub encb_dcen_cfg4, _: 2;
    pub encb_dcen_cfg5, _: 3;
    pub drv_enn, _: 4;
    pub enc_n_dco_cfg6, _: 5;
    pub sd_mode, _: 6;
    pub swcomp_in, _: 7;
    u8;
    pub version, _: 31, 24;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct FACTORY_CONF(u32);
    impl Debug;
    u8;
    pub fclktrim, set_fclktrim: 4, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct SHORT_CONF(u32);
    impl Debug;
    u8;
    pub s2vs_level, set_s2vs_level: 3, 0;
    pub s2g_level, set_s2g_level: 11, 8;
    pub shortfilter, set_shortfilter: 17, 16;
    pub shortdelay, set_shortdelay: 18;
}

bitfield! {
    #[derive(Clone, Copy, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct IHOLD_IRUN(u32);
    impl Debug;
    u8;
    pub ihold, set_ihold: 4, 0;
    pub irun, set_irun: 12, 8;
    pub ihold_delay, set_ihold_delay: 19, 16;
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct TPOWERDOWN(pub u32);

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct TSTEP(u32);
    impl Debug;
    u32;
    pub get, _: 19, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct TPWMTHRS(u32);
    impl Debug;
    u32;
    pub get, set: 19, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct VACTUAL(u32);
    impl Debug;
    i32;
    pub get, set: 23, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct TCOOLTHRS(u32);
    impl Debug;
    u32;
    pub get, set: 19, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct THIGH(u32);
    impl Debug;
    u32;
    pub get, set: 19, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct RAMPMODE(u32);
    impl Debug;
    u8;
    pub get, set: 1, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct XACTUAL(u32);
    impl Debug;
    i32;
    pub get, set: 31, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct VSTART(u32);
    impl Debug;
    u32;
    pub get, set: 17, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct A1(u32);
    impl Debug;
    u16;
    pub get, set: 15, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct V1(u32);
    impl Debug;
    u32;
    pub get, set: 19, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct AMAX(u32);
    impl Debug;
    u16;
    pub get, set: 15, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct VMAX(u32);
    impl Debug;
    u32;
    pub get, set: 22, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct DMAX(u32);
    impl Debug;
    u16;
    pub get, set: 15, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct D1(u32);
    impl Debug;
    u16;
    pub get, set: 15, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct VSTOP(u32);
    impl Debug;
    u32;
    pub get, set: 17, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct TZEROWAIT(u32);
    impl Debug;
    u16;
    pub get, set: 15, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct XTARGET(u32);
    impl Debug;
    i32;
    pub get, set: 31, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct VDCMIN(u32);
    impl Debug;
    u32;
    pub get, set: 22, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct SW_MODE(u32);
    impl Debug;
    u32;
    pub en_softstop, set_en_: 11;
    pub sg_stop, set_sg_stop: 10;
    pub en_latch_encoder, set_en_latch_encoder: 9;
    pub latch_r_inactive, set_latch_r_inactive: 8;
    pub latch_r_active, set_latch_r_active: 7;
    pub latch_l_inactive, set_latch_l_inactive: 6;
    pub latch_l_active, set_latch_l_active: 5;
    pub swap_lr, set_swap_lr: 4;
    pub pol_stop_r, set_pol_stop_r: 3;
    pub pol_stop_l, set_pol_stop_l: 2;
    pub stop_r_enable, set_stop_r_enable: 1;
    pub stop_l_enable, set_stop_l_enable: 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct RAMP_STAT(u32);
    impl Debug;
    u32;
    pub status_sg, set_status_sg:13;
    pub second_move, set_second_move:12;
    pub t_zerowait_active, set_t_zerowait_active:11;
    pub vzero, set_vzero:10;
    pub position_reached, set_position_reached:9;
    pub velocity_reached, set_velocity_reached:8;
    pub event_pos_reached, set_event_pos_reached:7;
    pub event_stop_sg, set_event_stop_sg:6;
    pub event_stop_r, set_event_stop_r:5;
    pub event_stop_l, set_event_stop_l:4;
    pub status_latch_r, set_status_latch_r:3;
    pub status_latch_l, set_status_latch_l:2;
    pub status_stop_r, set_status_stop_r:1;
    pub status_stop_l, set_status_stop_l:0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct XLATCH(u32);
    impl Debug;
    u32;
    pub get, set: 22, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct COOLCONF(u32);
    impl Debug;
    u16;
    pub semin, set_semin: 3, 0;
    pub seup, set_seup: 6, 5;
    pub semax, set_semax: 11, 8;
    pub sedn, set_sedn: 14, 13;
    pub seimin, set_seimin: 15;
    pub sgt, set_sgt: 22, 16;
    pub sfilt, set_sfilt: 24;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct MSCNT(u32);
    impl Debug;
    u16;
    pub get, _: 9, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct MSCURACT(u32);
    impl Debug;
    u16;
    pub cur_a, _: 8, 0;
    pub cur_b, _: 24, 16;
}

bitfield! {
    #[derive(Clone, Copy, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct CHOPCONF(u32);
    impl Debug;
    u32;
    pub toff, set_toff: 3, 0;
    pub hstrt, set_hstrt: 6, 4;
    pub hend, set_hend: 10, 7;
    pub fd3, set_fd3: 11;
    pub disfdcc, set_disfdcc: 12;
    pub chm, set_chm: 14;
    pub tbl, set_tbl: 16, 15;
    pub vhighfs, set_vhighfs: 18;
    pub vhighchm, set_vhighchm: 19;
    pub tpfd, set_tpfd: 23, 20;
    pub mres, set_mres: 27, 24;
    pub ntpol, set_intpol: 28;
    pub dedge, set_dedge: 29;
    pub diss2g, set_diss2g: 30;
    pub diss2vs, set_diss2vs: 31;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct DRV_CONF(u32);
    impl Debug;
    u8;
    pub bbmtime, set_bbmtime: 4, 0;
    pub bbmclks, set_bbmclks: 11, 8;
    pub otselect, set_otselect: 17, 16;
    pub drvstrength, set_drvstrength: 19, 18;
    pub filt_isense, set_filt_isense: 21, 20;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct GLOBAL_SCALER(u32);
    impl Debug;
    u8;
    pub get, set: 7, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct OFFSET_READ(u32);
    impl Debug;
    u8;
    pub phase_a, _: 15, 8;
    pub phase_b, _: 7, 0;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct DRV_STATUS(u32);
    impl Debug;
    u32;
    pub sg_result, _: 9, 0;
    pub s2vsa, _: 12;
    pub s2vsb, _: 13;
    pub stealth, _: 14;
    pub fsactive, _: 15;
    pub cs_actual, _: 20, 16;
    pub stallguard, _: 24;
    pub ot, _: 25;
    pub otpw, _: 26;
    pub s2ga, _: 27;
    pub s2gb, _: 28;
    pub ola, _: 29;
    pub olb, _: 30;
    pub stst, _: 31;
}

bitfield! {
    #[derive(Clone, Copy, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct PWMCONF(u32);
    impl Debug;
    u8;
    pub pwm_ofs, set_pwm_ofs: 7, 0;
    pub pwm_grad, set_pwm_grad: 15, 8;
    pub pwm_freq, set_pwm_freq: 17, 16;
    pub pwm_autoscale, set_pwm_autoscale: 18;
    pub pwm_autograd, set_pwm_autograd: 19;
    pub freewheel, set_freewheel: 21, 20;
    pub pwm_reg, set_pwm_reg: 27, 24;
    pub pwm_lim, set_pwm_lim: 31, 28;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct PWM_SCALE(u32);
    impl Debug;
    u8;
    pub pwm_scale_sum, _: 7, 0;
    u16;
    pub pwm_scale_auto, _: 24, 16;
}

bitfield! {
    #[derive(Clone, Copy, Default, Eq, Hash, PartialEq)]
    #[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
    #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
    #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
    pub struct PWM_AUTO(u32);
    impl Debug;
    u8;
    pub pwm_ofs_auto, _: 7, 0;
    pub pwm_grad_auto, _: 23, 16;
}

// Implementation Macros
// --------------------------------------------------------

/// A macro for generating `ReadableRegister` and `WritableRegister` implementations for the
/// register types based on the `R`, `W` or `RW` prefix.
macro_rules! impl_rw {
    (RW $T:ident) => {
        impl ReadableRegister for $T {}
        impl WritableRegister for $T {}
    };
    (R $T:ident) => {
        impl ReadableRegister for $T {}
    };
    (W $T:ident) => {
        impl WritableRegister for $T {}
    };
}

macro_rules! is_readable {
    (RW) => {
        true
    };
    (R) => {
        true
    };
    (W) => {
        false
    };
}

macro_rules! is_writable {
    (RW) => {
        true
    };
    (R) => {
        false
    };
    (W) => {
        true
    };
}

macro_rules! map_indices {
    ($ix:expr, $T:ident) => {
        pub(crate) const $T: usize = $ix;
    };
    ($ix:expr, $T:ident, $($Ts:ident),*) => {
        pub(crate) const $T: usize = $ix;
        map_indices!($T + 1, $($Ts),*);
    };
}

/// A macro for generating the `Address` enum along with the `Register` trait implementations.
macro_rules! impl_registers {
    ($($RW:ident $addr:literal $T:ident $map_access:ident $map_access_mut:ident,)*) => {
        /// Generate a private, unique index for each register into the `Map`'s inner array.
        mod map_index {
            map_indices!(0, $($T),*);
        }

        /// A dynamic representation of a register's 8-bit address.
        #[repr(u8)]
        #[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
        #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
        #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
        pub enum Address {
            $(
                $T = $addr,
            )*
        }

        /// A dynamic representation of a register's 32-bit state.
        #[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
        #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
        #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
        pub enum State {
            $(
                $T($T),
            )*
        }

        /// A map of the state of all registers in the TMC2209.
        #[derive(Clone, Debug, Eq, Hash, PartialEq)]
        #[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
        #[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
        pub struct Map {
            arr: MapArray,
        }

        /// The inner array storing all register state.
        ///
        /// Each register is laid out in the array in the order in which they are declared in the
        /// `impl_registers` macro. The `map_index` module is used internally to map register
        /// addresses and their state to the associated elements in the array.
        type MapArray = [State; COUNT];

        /// The total number of documented registers in the TMC2209.
        ///
        /// Useful for statically allocated register maps, etc.
        pub const COUNT: usize = 0 $(+ { let _ = Address::$T; 1 })*;

        impl Map {
            /// The total number of documented registers in the TMC2209.
            pub const LEN: usize = COUNT;

            /// Read-only access to the register of the given type.
            pub fn reg<T>(&self) -> &T
            where
                T: 'static + Register,
            {
                self.state(T::ADDRESS)
                    .reg::<T>()
                    // We gaurantee that `TmcRegisters` will always have state for each register, but need
                    // to avoid generating panicking branches, so we use an infinite loop rather than
                    // unwrap.
                    .unwrap_or_else(|_| loop {})
            }

            /// Mutable access to the register of the given type.
            pub fn reg_mut<T>(&mut self) -> &mut T
            where
                T: 'static + Register,
            {
                self.state_mut(T::ADDRESS)
                    .reg_mut::<T>()
                    // We gaurantee that `TmcRegisters` will always have state for each register, but need
                    // to avoid generating panicking branches, so we use an infinite loop rather than
                    // unwrap.
                    .unwrap_or_else(|_| loop {})
            }

            /// Read-only access to the dynamic representation of the register state at the given
            /// address.
            pub fn state(&self, addr: Address) -> &State {
                match addr {
                    $(
                        // We gaurantee that `Map` will always have state for each register.
                        Address::$T => unsafe {
                            self.arr.get_unchecked(map_index::$T)
                        }
                    )*
                }
            }

            /// Mutable access to the dynamic representation of the register state at the given
            /// address.
            ///
            /// Note: This should remain private for internal use only, as the user should never be
            /// allowed to change the stored `State` to a different variant.
            fn state_mut(&mut self, addr: Address) -> &mut State {
                match addr {
                    $(
                        // We gaurantee that `Map` will always have state for each register.
                        Address::$T => unsafe {
                            self.arr.get_unchecked_mut(map_index::$T)
                        }
                    )*
                }
            }

            /// Update the given register state.
            pub fn set_state(&mut self, state: State) {
                *self.state_mut(state.addr()) = state;
            }

            // Generate the short-hand names for gaining direct access to typed register state.
            $(
                pub fn $map_access(&self) -> &$T {
                    self.reg::<$T>()
                }

                pub fn $map_access_mut(&mut self) -> &mut $T {
                    self.reg_mut::<$T>()
                }
            )*
        }

        impl Address {
            /// All register addresses.
            pub const ALL: &'static [Self] = &[
                $(
                    Self::$T,
                )*
            ];

            /// Whether or not we can send a read request to the register address.
            pub fn readable(&self) -> bool {
                match *self {
                    $(
                        Self::$T => is_readable!($RW),
                    )*
                }
            }

            /// Whether or not we can send a write request to the register address.
            pub fn writable(&self) -> bool {
                match *self {
                    $(
                        Self::$T => is_writable!($RW),
                    )*
                }
            }
        }

        impl State {
            /// Construct a register state from its address and data represented as a `u32`.
            pub fn from_addr_and_data(addr: Address, data: u32) -> Self {
                match addr {
                    $(
                        Address::$T => State::$T(<_>::from(data)),
                    )*
                }
            }

            /// Construct the default register state associated with the given address.
            pub fn from_addr_default(addr: Address) -> Self {
                match addr {
                    $(
                        Address::$T => State::$T(<_>::default()),
                    )*
                }
            }

            /// The address of the register with which this state is associated.
            pub fn addr(&self) -> Address {
                match *self {
                    $(
                        State::$T(_) => Address::$T,
                    )*
                }
            }

            /// Attempt to retrieve a reference to a register of type `R` from the dynamic register
            /// `State` representation.
            ///
            /// Returns an `Err` if the register type does not match.
            pub fn reg<R>(&self) -> Result<&R, UnexpectedAddress>
            where
                R: 'static + Register,
            {
                match *self {
                    $(
                        Self::$T(ref r) => (r as &dyn core::any::Any)
                            .downcast_ref()
                            .ok_or(UnexpectedAddress),
                    )*
                }
            }

            /// Attempt to retrieve a mutable reference to a register of type `R` from the dynamic
            /// register `State` representation.
            ///
            /// Returns an `Err` if the register type does not match.
            pub fn reg_mut<R>(&mut self) -> Result<&mut R, UnexpectedAddress>
            where
                R: 'static + Register,
            {
                match *self {
                    $(
                        Self::$T(ref mut r) => (r as &mut dyn core::any::Any)
                            .downcast_mut()
                            .ok_or(UnexpectedAddress),
                    )*
                }
            }
        }

        impl Default for Map {
            fn default() -> Self {
                let arr = [$(
                    State::$T($T::default()),
                )*];
                Map { arr }
            }
        }

        impl core::ops::Deref for Map {
            type Target = MapArray;
            fn deref(&self) -> &Self::Target {
                &self.arr
            }
        }

        #[cfg(feature = "hash")]
        impl hash32::Hash for Address {
            fn hash<H>(&self, state: &mut H)
            where
                H: hash32::Hasher,
            {
                (*self as u8).hash(state)
            }
        }

        #[cfg(feature = "hash")]
        impl hash32::Hash for State {
            fn hash<H>(&self, state: &mut H)
            where
                H: hash32::Hasher,
            {
                let u: u32 = (*self).into();
                u.hash(state)
            }
        }

        impl core::ops::Index<Address> for Map {
            type Output = State;
            fn index(&self, addr: Address) -> &Self::Output {
                self.state(addr)
            }
        }

        impl core::ops::IndexMut<Address> for Map {
            fn index_mut(&mut self, addr: Address) -> &mut Self::Output {
                self.state_mut(addr)
            }
        }

        impl Into<u8> for Address {
            fn into(self) -> u8 {
                self as u8
            }
        }

        impl Into<u32> for State {
            fn into(self) -> u32 {
                match self {
                    $(
                        State::$T(r) => r.into(),
                    )*
                }
            }
        }

        impl core::convert::TryFrom<u8> for Address {
            type Error = UnknownAddress;
            fn try_from(u: u8) -> Result<Self, Self::Error> {
                let reg = match u {
                    $(
                        $addr => Self::$T,
                    )*
                    _ => return Err(UnknownAddress),
                };
                Ok(reg)
            }
        }

        $(
            impl From<u32> for $T {
                fn from(u: u32) -> $T {
                    $T(u)
                }
            }

            impl From<$T> for State {
                fn from(r: $T) -> Self {
                    State::$T(r)
                }
            }

            impl Into<u32> for $T {
                fn into(self) -> u32 {
                    self.0 as u32
                }
            }

            impl Register for $T {
                const ADDRESS: Address = Address::$T;
            }

            impl core::convert::TryFrom<State> for $T {
                type Error = UnexpectedAddress;
                fn try_from(state: State) -> Result<Self, Self::Error> {
                    match state {
                        State::$T(s) => Ok(s),
                        _ => Err(UnexpectedAddress),
                    }
                }
            }
        )*

        $(
            impl_rw!{$RW $T}
        )*
    };
}

// Register Implementations
// --------------------------------------------------------

impl_registers! {
    // General Registers.
    RW 0x00 GCONF gconf gconf_mut,
    RW 0x01 GSTAT gstat gstat_mut,
    R  0x02 IFCNT ifcnt ifcnt_mut,
    W  0x03 SLAVECONF slaveconf slaveconf_mut,
    R  0x04 IOIN ioin ioin_mut,
    //W 0x04 0x04 1 OUTPUT   Sets the IO output pin polarity in UART mode
    //W 0x05 32 X_COMPARE    Position comparison register for motion controller position
    W  0x06 OTP_PROG otp_prog otp_prog_mut,
    R  0x07 OTP_READ otp_read otp_read_mut,
    RW 0x08 FACTORY_CONF factory_conf factory_conf_mut,
    W 0x09 SHORT_CONF short_conf short_conf_mut,
    W 0x0A DRV_CONF drv_conf drv_conf_mut,
    W 0x0B GLOBAL_SCALER global_scaler global_scaler_mut,
    R 0x0C OFFSET_READ offset_read offset_read_mut,

    // Velocity Dependent Control.
    W  0x10 IHOLD_IRUN ihold_irun ihold_irun_mut,
    W  0x11 TPOWERDOWN tpowerdown tpowerdown_mut,
    R  0x12 TSTEP tstep tstep_mut,
    W  0x13 TPWMTHRS tpwmthrs tpwmthrs_mut,
    W 0x15 THIGH thigh thigh_mut,

    // Ramp Generator Motion Control Register Set
    RW 0x20 RAMPMODE rampmode rampmode_mut,
    RW 0x21 XACTUAL xactual xactual_mut,
    R 0x22 VACTUAL vactual vactual_mut,
    W 0x23 VSTART vstart vstart_mut,
    W 0x24 A1 a1 a1_mut,
    W 0x25 V1 v1 v1_mut,
    W 0x26 AMAX amax amax_mut,
    W 0x27 VMAX vmax vmax_mut,
    W 0x28 DMAX dmax dmax_mut,
    W 0x2A D1 d1 d1_mut,
    W 0x2B VSTOP vstop vstop_mut,
    W 0x2C TZEROWAIT tzerowait tzerowait_mut,
    RW 0x2D XTARGET xtarget xtarget_mut,

    // Ramp Generator Driver Feature Control Register Set
    W 0x33 VDCMIN vdcmin vdcmin_mut,
    RW 0x34 SW_MODE sw_mode sw_mode_mut,
    RW 0x35 RAMP_STAT ramp_stat ramp_stat_mut,	// R+WC
    R 0x36 XLATCH xlatch xlatch_mut,

    // Encoder Register
    //RW 0x38 11 ENCMODE
    //RW 0x39 32 X_ENC
    //W 0x3A 32 ENC_CONST
    //R+WC 0x3B 2 ENC_STATUS
    //R 0x3C 32 ENC_LATCH
    //W 0x3D 20 ENC_DEVIATION

    // Motor Driver Registers
    //W 0x60 32 MSLUT[0]
    //W 0x61…0x67 MSLUT[1...7]
    //W 0x68 MSLUTSEL
    //W 0x69 MSLUTSTART

    // StallGuard Control.
    W  0x14 TCOOLTHRS tcoolthrs tcoolthrs_mut,

    // Sequencer Registers.
    R  0x6A MSCNT mscnt mscnt_mut,
    R  0x6B MSCURACT mscuract mscuract_mut,

    // Chopper Control Registers.
    RW 0x6C CHOPCONF chopconf chopconf_mut,
    W  0x6D COOLCONF coolconf coolconf_mut,
    //W 0x6E 24 DCCTRL

    R  0x6F DRV_STATUS drv_status drv_status_mut,
    RW 0x70 PWMCONF pwmconf pwmconf_mut,
    R  0x71 PWM_SCALE pwm_scale pwm_scale_mut,
    R  0x72 PWM_AUTO pwm_auto pwm_auto_mut,
    //R 0x73 LOST_STEPS
}

impl VACTUAL {
    /// Creates the `VACTUAL` register enabled for UART control but in a stopped state.
    pub const ENABLED_STOPPED: Self = VACTUAL(1);
}

// Default Register States (taken from TMC-API reference).
// --------------------------------------------------------

impl Default for GCONF {
    fn default() -> Self {
        Self(0)
    }
}

impl Default for IHOLD_IRUN {
    fn default() -> Self {
        Self(0x00001F00)
    }
}

impl Default for CHOPCONF {
    fn default() -> Self {
        Self(0x10000053)
    }
}

impl Default for PWMCONF {
    fn default() -> Self {
        Self(0xC10D0024)
    }
}

impl Default for TPOWERDOWN {
    fn default() -> Self {
        Self(20)
    }
}

// Sanity Checks
// --------------------------------------------------------

#[test]
fn test_slaveconf() {
    let mut s = SLAVECONF(0);
    assert_eq!(s.0, 0b000000000000);
    s.set_senddelay(15);
    assert_eq!(s.0, 0b111100000000);
}

#[test]
fn test_gconf() {
    let mut g = GCONF(0);
    // assert_eq!(g.0, 0b0000000000);
    // g.set_i_scale_analog(true);
    // assert_eq!(g.0, 0b0000000001);
    // g.set_test_mode(true);
    // assert_eq!(g.0, 0b1000000001);
    // g = Default::default();
    // assert_eq!(g.0, 0x00000041);
}
