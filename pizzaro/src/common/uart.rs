use super::global_timer::{now, AtomiDuration, AtomiInstant, Delay};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use embedded_hal::serial::Read;
use futures::FutureExt;
use generic::atomi_error::AtomiError;

struct UartRead<'a, T: Read<u8>> {
    uart: &'a mut T,
    buffer: &'a mut [u8],
    pos: usize, // 用于跟踪已读取的字节位置
}

struct UartReadTimeout<'a, T: Read<u8>> {
    uart_read: UartRead<'a, T>,
    delay: Option<Delay>,
    timeout: Option<AtomiDuration>,
    start: AtomiInstant,
}

impl<'a, T: Read<u8>> Future for UartRead<'a, T> {
    type Output = Result<(), T::Error>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        while this.pos < this.buffer.len() {
            match this.uart.read() {
                Ok(byte) => {
                    // info!("UartRead::poll() got byte: {}, pos = {}, len = {}", byte, this.pos, this.buffer.len());
                    this.buffer[this.pos] = byte;
                    this.pos += 1; // 更新读取位置
                }
                Err(nb::Error::WouldBlock) => {
                    cx.waker().wake_by_ref();
                    return Poll::Pending;
                }
                Err(nb::Error::Other(e)) => return Poll::Ready(Err(e)),
            }
        }
        Poll::Ready(Ok(()))
    }
}

impl<'a, T: Read<u8>> Future for UartReadTimeout<'a, T> {
    type Output = Result<(), AtomiError>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();

        if let Some(timeout) = this.timeout {
            if let Some(duration) = now().checked_duration_since(this.start) {
                if duration >= timeout {
                    return Poll::Ready(Err(AtomiError::UartReadError)); // 返回超时错误
                }
            }
        }

        if let Some(ref mut delay) = this.delay {
            match Pin::new(delay).poll(cx) {
                Poll::Ready(()) => return Poll::Ready(Err(AtomiError::UartReadError)), // 返回超时错误
                Poll::Pending => {}
            }
        }
        match this.uart_read.poll_unpin(cx) {
            Poll::Ready(Ok(())) => Poll::Ready(Ok(())),
            Poll::Ready(Err(_)) => Poll::Ready(Err(AtomiError::UartReadError)),
            Poll::Pending => Poll::Pending,
        }
    }
}

pub async fn uart_read<'a, T: Read<u8>>(
    uart: &'a mut T,
    buffer: &'a mut [u8],
) -> Result<(), T::Error> {
    UartRead {
        uart,
        buffer,
        pos: 0,
    }
    .await
}

pub async fn uart_read_timeout<'a, T: Read<u8>>(
    uart: &'a mut T,
    buffer: &'a mut [u8],
    timeout: Option<AtomiDuration>,
) -> Result<(), AtomiError> {
    let delay = timeout.map(Delay::new);
    UartReadTimeout {
        uart_read: UartRead {
            uart,
            buffer,
            pos: 0,
        },
        delay,
        timeout,
        start: now(),
    }
    .await
}
