use defmt::Format;

pub const MAX_TEXT_LENGTH: usize = 1024;

#[derive(Copy, Clone, Debug, Format)]
pub enum TouchScreenError {
    UnknownObject,
    WouldBlock,
    InvalidFrame,
    FromBeBytesError,
    FromUtf8Error,
}

#[derive(Copy, Clone, Debug, Format)]
pub enum TouchScreenEnum {
    // button
    Button {
        screen_id: u16,
        object_id: u16,
        clicked: bool,
    },
    Slider {
        screen_id: u16,
        object_id: u16,
        value: u32,
    },
    Text {
        screen_id: u16,
        object_id: u16,
        content: [u8; MAX_TEXT_LENGTH],
    },
}

pub const DEFAULT_SCREEN_ID: u16 = 0;

#[repr(u16)]
pub enum TouchScreenObjectId {
    BtnEngage = 1,
    BtnStop = 2,
    SliderBeltPosition = 3,
    SliderBeltSpeed = 9,
    SliderDisperser1 = 8,
    SliderDisperser2 = 12,
    SliderMainRotary = 13,
    SliderPresserPosition = 4,

    // FIXME: TBD
    TextWeight1 = 14,
    TextWeight2 = 15,
    TextWeight3 = 16,
    TextWeight4 = 17,
}

impl TouchScreenObjectId {
    pub fn as_u16(self) -> u16 {
        self as u16
    }
}

impl TouchScreenEnum {
    pub fn new_button(screen_id: u16, object_id: u16, clicked: bool) -> Self {
        Self::Button {
            screen_id,
            object_id,
            clicked,
        }
    }
    pub fn new_text(screen_id: u16, object_id: u16, content: &str) -> Self {
        let mut content_bytes = [0u8; MAX_TEXT_LENGTH];
        content_bytes.copy_from_slice(content.as_bytes());
        Self::Text {
            screen_id,
            object_id,
            content: content_bytes,
        }
    }
    pub fn new_slider(screen_id: u16, object_id: u16, value: u32) -> Self {
        Self::Slider {
            screen_id,
            object_id,
            value,
        }
    }

    /// 设置控件内容的请求
    pub fn to_update(&self, buf: &mut [u8]) -> Result<usize, TouchScreenError> {
        buf[..3].copy_from_slice(&[0xee, 0xb1, 0x10]);
        let size = match self {
            Self::Button {
                screen_id,
                object_id,
                clicked,
            } => {
                //EE 【B1 10 Screen_id Control_id Status 】FF FC FF FF
                // Screen_id(2 个字节)：画面编号
                // Control_id(2 个字节)：按钮控件编号
                // Status(1 个字节) ：按钮状态
                // 0x00：设置按钮由按下变成弹起状态
                // 0x01：设置按钮从弹起变成按下状态
                buf[3..5].copy_from_slice(&screen_id.to_be_bytes());
                buf[5..7].copy_from_slice(&object_id.to_be_bytes());
                buf[8] = if *clicked { 1 } else { 0 };
                8
            }
            Self::Slider {
                screen_id,
                object_id,
                value,
            } => {
                //EE 【B1 10 Screen_id Control_id Slidervalue】 FF FC FF FF
                // Screen_id(2 个字节)：画面编号
                // Control_id(2 个字节)：控件编号
                // Progressvalue（4 个字节）：新的进度条值
                buf[3..5].copy_from_slice(&screen_id.to_be_bytes());
                buf[5..7].copy_from_slice(&object_id.to_be_bytes());
                buf[7..11].copy_from_slice(&value.to_be_bytes());
                11
            }
            Self::Text {
                // 更新文本框
                screen_id,
                object_id,
                content,
            } => {
                //EE【B1 10 Screen_id Control_id Strings 】FF FC FF FF
                // Screen_id(2 个字节)：画面编号
                // Control_id(2 个字节)：按钮控件编号
                // Strings（不定长）：用户写入的字符串,0结尾

                buf[3..5].copy_from_slice(&screen_id.to_be_bytes());
                buf[5..7].copy_from_slice(&object_id.to_be_bytes());
                let content_len = content
                    .iter()
                    .position(|&x| x == 0)
                    .unwrap_or(content.len());
                buf[7..7 + content_len].copy_from_slice(&content[..content_len]);
                buf[7 + content.len()] = 0; // 字符串以0结尾
                7 + content.len() + 1
            }
        };
        buf[size..size + 4].copy_from_slice(&[0xff, 0xfc, 0xff, 0xff]);
        Ok(size + 4)
    }

    /// 获取控件内容的请求
    pub fn to_read(&self, buf: &mut [u8]) -> Result<usize, TouchScreenError> {
        // 所有读取控件内容的指令结构都是一模一样的。
        buf[..3].copy_from_slice(&[0xee, 0xb1, 0x11]);
        let (screen_id, object_id) = match self {
            Self::Button {
                screen_id,
                object_id,
                clicked: _clicked,
            } => (screen_id, object_id),
            Self::Slider {
                screen_id,
                object_id,
                value: _value,
            } => (screen_id, object_id),
            Self::Text {
                // 更新文本框
                screen_id,
                object_id,
                content: _content,
            } => (screen_id, object_id),
        };
        buf[3..5].copy_from_slice(&screen_id.to_be_bytes());
        buf[5..7].copy_from_slice(&object_id.to_be_bytes());
        buf[7..11].copy_from_slice(&[0xff, 0xfc, 0xff, 0xff]);
        Ok(11)
    }

    /// 解析为控件对象
    pub fn parse(frame: &[u8]) -> Result<Self, TouchScreenError> {
        if frame.len() < 6
            || frame[0] != 0xEE
            || frame[frame.len() - 4..] != [0xFF, 0xFC, 0xFF, 0xFF]
        {
            return Err(TouchScreenError::InvalidFrame);
        }

        // 提取指令
        let _instruction = frame[1];
        let screen_id = u16::from_be_bytes(
            frame[3..5]
                .try_into()
                .map_err(|_| TouchScreenError::FromBeBytesError)?,
        );
        let object_id = u16::from_be_bytes(
            frame[5..7]
                .try_into()
                .map_err(|_| TouchScreenError::FromBeBytesError)?,
        );
        let control_type = frame[7];
        // defmt::info!("parsing... {}, control_type:{}", frame, control_type);

        match control_type {
            0x13 => {
                // Slider控件
                let value = u32::from_be_bytes(
                    frame[8..12]
                        .try_into()
                        .map_err(|_| TouchScreenError::FromBeBytesError)?,
                );
                Ok(Self::Slider {
                    screen_id,
                    object_id,
                    value,
                })
            }
            0x10 => {
                // 按钮控件
                let value = frame[9];
                Ok(Self::Button {
                    screen_id,
                    object_id,
                    clicked: value == 0x1,
                })
            }
            0x11 => {
                // 文本控件
                let value = &frame[8..frame.len() - 4];
                let mut content_buf = [0u8; MAX_TEXT_LENGTH];
                content_buf.copy_from_slice(value);
                Ok(Self::Text {
                    screen_id,
                    object_id,
                    content: content_buf,
                })
            }
            _ => Err(TouchScreenError::UnknownObject),
        }
    }
}
