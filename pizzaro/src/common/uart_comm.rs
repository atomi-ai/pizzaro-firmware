use alloc::vec;

use defmt::{info, Debug2Format, Format};
use embedded_hal::serial::{Read, Write};
use serde::{Deserialize, Serialize};

use generic::atomi_error::AtomiError;

use crate::common::uart::uart_read;

use super::{global_timer::AtomiDuration, uart::uart_read_timeout};

pub struct UartComm<'a, T: Read<u8> + Write<u8>> {
    uart: &'a mut T,
    expected_response_length: usize,
}

impl<'a, T: Read<u8> + Write<u8>> UartComm<'a, T> {
    pub fn new(uart: &'a mut T, expected_response_length: usize) -> Self {
        UartComm {
            uart,
            expected_response_length,
        }
    }

    fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), AtomiError> {
        for word in buffer {
            nb::block!(self.uart.write(word.clone())).map_err(|_| AtomiError::UartWriteError)?;
        }

        Ok(())
    }

    pub fn send<U: Format + Serialize>(&mut self, message: U) -> Result<(), AtomiError> {
        let out = postcard::to_allocvec::<U>(&message).map_err(|_| AtomiError::UartInvalidInput)?;

        info!(
            "Send data: ({}, {}), original = {}",
            out.len(),
            Debug2Format(&out),
            message
        );
        // 发送长度和数据
        // TODO(zephyr): 看看怎么wrap T::Error到PizzaroError里面去.
        self.bwrite_all(&[out.len() as u8])
            .map_err(|_| AtomiError::UartWriteError)?;
        self.bwrite_all(&out)
            .map_err(|_| AtomiError::UartWriteError)
    }

    pub async fn try_recv<U, F, E>(
        &mut self,
        processor: F,
        timeout: Option<AtomiDuration>,
    ) -> Result<U, AtomiError>
    where
        F: Fn(&[u8]) -> Option<U>,
        E: Into<AtomiError>,
    {
        let mut length_buffer = [0u8; 1];

        loop {
            match uart_read_timeout(self.uart, &mut length_buffer, timeout).await {
                Ok(()) => {
                    if let Some(result) = processor(&length_buffer) {
                        return Ok(result);
                    } else if timeout.is_some() {
                        return Err(AtomiError::IgnoredMsg);
                    }
                }
                Err(_) => return Err(AtomiError::UartReadError),
            }
        }
    }

    pub async fn recv<U>(&mut self) -> Result<U, AtomiError>
    where
        U: for<'b> Deserialize<'b>,
    {
        // 读取响应长度
        let mut length_buffer = [0u8; 1];
        uart_read(self.uart, &mut length_buffer)
            .await
            .map_err(|_| AtomiError::UartReadError)?;

        let response_length = length_buffer[0] as usize;
        if response_length == 0 || response_length > self.expected_response_length {
            return Err(AtomiError::UartInvalidData);
        }
        // info!("UartComm::recv() 3: got length = {}", response_length);

        // 读取响应数据
        let mut response_buffer = vec![0; response_length];
        uart_read(self.uart, &mut response_buffer)
            .await
            .map_err(|_| AtomiError::UartReadError)?;

        info!(
            "UartComm::recv() 6: got data = {}",
            Debug2Format(&response_buffer)
        );
        postcard::from_bytes::<U>(&response_buffer).map_err(|_| AtomiError::UartInvalidData)
    }
}
