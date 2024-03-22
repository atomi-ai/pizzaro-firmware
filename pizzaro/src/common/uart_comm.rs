use alloc::vec;

use defmt::{debug, error, Debug2Format, Format};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::serial::{Read, Write};
use futures::future::{select, Either};
use futures::pin_mut;
use serde::{Deserialize, Serialize};

use crate::common::global_timer::{AtomiDuration, Delay};
use generic::atomi_error::AtomiError;

use crate::common::uart::uart_read;

pub struct UartComm<'a, D: OutputPin, T: Read<u8> + Write<u8>> {
    uart: &'a mut T,
    dir_pin: &'a mut Option<D>,
    expected_response_length: usize,
}

impl<'a, D: OutputPin, T: Read<u8> + Write<u8>> UartComm<'a, D, T> {
    pub fn new(
        uart: &'a mut T,
        dir_pin: &'a mut Option<D>,
        expected_response_length: usize,
    ) -> Self {
        if let Some(n_re) = dir_pin {
            n_re.set_low().map_err(|_| AtomiError::UartSetDirError).unwrap();
        }
        UartComm { uart, dir_pin, expected_response_length }
    }

    fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), AtomiError> {
        for word in buffer {
            nb::block!(self.uart.write(*word)).map_err(|_| AtomiError::UartWriteError)?;
        }

        Ok(())
    }

    pub fn send<U: Format + Serialize>(&mut self, message: U) -> Result<(), AtomiError> {
        let out = postcard::to_allocvec::<U>(&message).map_err(|_| AtomiError::UartInvalidInput)?;

        debug!("Send data: ({}, {}), original = {}", out.len(), Debug2Format(&out), message);
        // 发送长度和数据
        // TODO(zephyr): 看看怎么wrap T::Error到PizzaroError里面去.

        if let Some(n_re) = self.dir_pin {
            n_re.set_high().map_err(|_| AtomiError::UartSetDirError).unwrap();
        }

        // let bw_res0 = self
        //     .bwrite_all(&[out.len() as u8])
        //     .map_err(|_| AtomiError::UartWriteError);
        // let bw_res = if bw_res0.is_ok() {
        //     self.bwrite_all(&out)
        //         .map_err(|_| AtomiError::UartWriteError)
        // } else {
        //     bw_res0
        // };

        let res = (|| {
            self.bwrite_all(&[out.len() as u8]).map_err(|_| AtomiError::UartWriteError)?;
            self.bwrite_all(&out).map_err(|_| AtomiError::UartWriteError)?;
            nb::block!(self.uart.flush()).map_err(|_| AtomiError::UartFlushError)
        })();

        if let Some(n_re) = self.dir_pin {
            n_re.set_low().map_err(|_| AtomiError::UartSetDirError).unwrap();
        }
        res
    }

    pub async fn recv_timeout<U>(&mut self, timeout: AtomiDuration) -> Result<U, AtomiError>
    where
        U: for<'b> Deserialize<'b>,
    {
        let recv_future = self.recv();
        let timeout_future = Delay::new(timeout);
        pin_mut!(recv_future);
        pin_mut!(timeout_future);
        match select(recv_future, timeout_future).await {
            Either::Left((uart_result, _)) => uart_result,
            Either::Right(_) => Err(AtomiError::UartReadTimeout),
        }
    }

    pub async fn recv<U>(&mut self) -> Result<U, AtomiError>
    where
        U: for<'b> Deserialize<'b>,
    {
        // 读取响应长度
        let mut length_buffer = [0u8; 1];

        // retry 3 times to make sure you read the correct length
        let mut correct = false;
        for _ in 0..3 {
            if uart_read(self.uart, &mut length_buffer).await.is_ok() {
                correct = true;
                break;
            }
            debug!("Errors in reading UART's first byte, try again");
        }
        if !correct {
            error!("Errors in reading UART");
            return Err(AtomiError::UartReadError);
        }

        let response_length = length_buffer[0] as usize;
        if response_length == 0 || response_length > self.expected_response_length {
            return Err(AtomiError::UartInvalidData);
        }
        // info!("UartComm::recv() 3: got length = {}", response_length);

        // 读取响应数据
        let mut response_buffer = vec![0; response_length];
        uart_read(self.uart, &mut response_buffer).await.map_err(|_| AtomiError::UartReadError)?;

        debug!("UartComm::recv() 6: got data = {}", Debug2Format(&response_buffer));
        postcard::from_bytes::<U>(&response_buffer).map_err(|_| AtomiError::UartInvalidData)
    }
}
