use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use embedded_hal::serial::Read;

struct UartRead<'a, T: Read<u8>> {
    uart: &'a mut T,
    buffer: &'a mut [u8],
    pos: usize, // 用于跟踪已读取的字节位置
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
                },
                Err(nb::Error::WouldBlock) => {
                    cx.waker().wake_by_ref();
                    return Poll::Pending;
                },
                Err(nb::Error::Other(e)) => return Poll::Ready(Err(e)),
            }
        }

        Poll::Ready(Ok(()))
    }
}

pub async fn uart_read<'a, T: Read<u8>>(uart: &'a mut T, buffer: &'a mut [u8]) -> Result<(), T::Error> {
    UartRead { uart, buffer, pos: 0 }.await
}
