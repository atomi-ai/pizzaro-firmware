//! 处理串口屏，async版
use crate::uart::{UartAsyncHelperInterface, UartError, UartProcessor, UartSerdeInterface};

use super::touch_screen::{TouchScreenEnum, TouchScreenError};
use super::uart::UartPeripheralInterface;
use crate::constants::TOUCH_SCREEN_BUFFER_LENGTH;
use common::global_timer::Delay;
use fugit::ExtU64;

pub struct TouchScreenProcessor<I: UartPeripheralInterface>(
    UartProcessor<I, TouchScreenEnum, TOUCH_SCREEN_BUFFER_LENGTH>,
);

impl UartSerdeInterface<TouchScreenEnum> for TouchScreenEnum {
    fn load(buffer: &[u8]) -> Result<(TouchScreenEnum, usize), UartError> {
        if buffer.len() >= 4 && buffer[buffer.len() - 4..] == [0xFF, 0xFC, 0xFF, 0xFF] {
            match TouchScreenEnum::parse(buffer) {
                Ok(result) => Ok((result, buffer.len())),
                Err(TouchScreenError::WouldBlock) => Err(UartError::WouldBlock),
                Err(e) => Err(UartError::NestedTouchScreenError(e)),
            }
        } else {
            Err(UartError::WouldBlock)
        }
    }

    fn dump(evt: TouchScreenEnum, buffer: &mut [u8]) -> Result<usize, UartError> {
        evt.to_update(buffer).map_err(|e| {
            if matches!(e, TouchScreenError::WouldBlock) {
                UartError::WouldBlock
            } else {
                UartError::NestedTouchScreenError(e)
            }
        })
    }
}

impl<I> UartAsyncHelperInterface<TouchScreenEnum> for TouchScreenProcessor<I>
where
    I: UartPeripheralInterface,
{
    fn emit(&mut self) -> Option<TouchScreenEnum> {
        self.0.pull_evt.take()
    }

    fn collect(&mut self, evt: TouchScreenEnum) -> Result<(), UartError> {
        if self.0.push_evt.is_none() {
            self.0.push_evt = Some(evt);
            Ok(())
        } else {
            Err(UartError::WouldBlock)
        }
    }
}

impl<I: UartPeripheralInterface> TouchScreenProcessor<I> {
    pub fn new(uart: I) -> Self {
        Self(UartProcessor::new(uart))
    }

    pub fn process(&mut self) {
        self.0.process()
    }
}

pub struct TouchScreenAdapter<T: UartPeripheralInterface + 'static>(
    &'static mut TouchScreenProcessor<T>,
);

impl<I: UartPeripheralInterface> TouchScreenAdapter<I> {
    pub fn new(ts: &'static mut TouchScreenProcessor<I>) -> Self {
        Self(ts)
    }

    pub fn process(&mut self) {
        critical_section::with(|_cs| self.0.process())
    }

    pub fn emit(&mut self) -> Option<TouchScreenEnum> {
        critical_section::with(|_cs| self.0.emit())
    }

    pub fn collect(&mut self, evt: TouchScreenEnum) -> Result<(), UartError> {
        critical_section::with(|_cs| self.0.collect(evt))
    }
}

pub async fn touch_screen_tick<T: UartPeripheralInterface>(mut ts: TouchScreenAdapter<T>) {
    loop {
        ts.process();
        Delay::new(1.millis()).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_touch_screen_enums() {
        // let mut processor = TouchScreenProcessor::new();
        let mut buf = [0u8; 1024];
        let size = TouchScreenEnum::new_button(1, 2, true)
            .to_update(&buf)
            .unwrap();
        assert_eq!(
            buf[..size],
            [0xee, 0xb1, 0x10, 0x00, 0x01, 0x00, 0x02, 0x01, 0xff, 0xfc, 0xff, 0xff]
        );
    }
}
