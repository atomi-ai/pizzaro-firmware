macro_rules! config {
    (
        $(
            $key:ident : $value:expr
        ),+ $(,)?
    ) => {
        $(
            pub const $key: bool = $value;
        )+
    };
}

macro_rules! parameters {
    (
        $(
            $key:ident : $type:ty = $value:expr
        ),+ $(,)?
    ) => {
        $(
            pub const $key: $type = $value;
        )+
    };
}

config! {
    // for MMD
    REVERT_MMD_BR_DIRECTION: false,
    REVERT_MMD_EN_SIGNAL: true,
    REVERT_MMD_BL0_DIRECTION: false,
    REVERT_MMD_BL1_DIRECTION: false,
    REVERT_MMD_STEPPER42_0_DIRECTION: false,
    REVERT_MMD_STEPPER42_1_DIRECTION: false,
    REVERT_MMD_STEPPER57_DIRECTION: false,
    SWITCH_MMD_LIMIT_SWITCHS: false,
    SWITCH_MMD_PROXIMITY_SENSORS: false,
    // for HPD
    REVERT_HPD_BR_DIRECTION: true, // ok
    REVERT_HPD_EN_SIGNAL: true,	    // ok
    REVERT_HPD_LINEARSCALE_DIRECTION: true, // ok
}

parameters! {
    LINEAR_BULL: (f32, f32, f32)= (0.013, 0.0, 0.0), // ok
    // 左转最高速度，左转最小速度，右转最小速度，右转最高速度
    HPD_BR_THRESHOLD: (f32, f32, f32, f32) = (0.03, 0.45, 0.55, 0.97),			   // ok
    // 定位误差，pid和机械部分调的足够好之后，可以不需要这个误差值
    // HPD_POSITION_ERROR: u32 = 100,
    // for MC
    WEIGHT_SENSOR0_IDX: usize = 0,
    WEIGHT_SENSOR1_IDX: usize = 1,
    WEIGHT_SENSOR2_IDX: usize = 2,
    WEIGHT_SENSOR3_IDX: usize = 3,
}
