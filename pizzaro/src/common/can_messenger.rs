/*
 在pizzaro项目中，我们将使用CAN 2.0B作为我们的CAN通讯标准，下面是我们使用extended-id的方法：
 - 29位extended-id的结构可以分解为：
   - 主被动位：1-bit, 用于标识这个frame是由发送方标识的，还是接收方标识的。发送方标识时，此位为1，反之为0。
   - to can id: 8-bit，这个frame发送的目标
   - from can id: 8-bit，这个frame发送方
   - sequence number: 12-bit，这个frame对应的sequence number。
 - 主被动位也可以理解为请求/响应位，虽然有时候不作为请求响应的功能体现，所以我们称呼它位主被动位。如果
 从请求/响应的角度理解的话，如果它是一个请求，这个frame里的seq-no就是由发送端标识的；如果它是一个响应，
 seq-no就会有接收端标识。
   举个例子可能比较好理解：
   - 譬如我有几个node，对应的can-id如下： MC - 1 / MMD - 2 / HPD - 3
   - 当前它们各自的seq-no为： MC - 0x011 / MMD - 0x122 / HPD - 0x233
   - 那么当MC => HPD发请求并期待响应时，它发的请求的id是0x10301011，它等待的响应id应该为0x00103011。
   - 如果是HPD => MC发送一个消息，它的id会是0x10103233。
 基于这个例子，我们可以知道，同样从HPD=>MC发送消息，它使用的id会有比较大差别。我们可以使用这种方法来等待
 对应请求的响应。

 这里seq-no用了12-bit有点多。我们暂时没有其它需求，以后有需求的时候，可以考虑再分8-bit出去给别的需求，
 感觉4~8 bits用于seq-no已经够了，要不系统里面将有大量请求在并发执行，也许就过于复杂了。
*/
use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use can2040::{Can2040, CanFrame};
use critical_section::Mutex;
use defmt::Debug2Format;
use embedded_can::Id::Extended;
use embedded_can::{ExtendedId, Frame, Id};
use fugit::ExtU64;
use heapless::{FnvIndexMap, FnvIndexSet};
use serde::{Deserialize, Serialize};

use generic::atomi_error::AtomiError;

use crate::common::global_timer::Delay;
use crate::common::message_queue::{MessageQueueInterface, MessageQueueWrapper};
use crate::common::once::Once;

const PASSIVE_BIT: u32 = 0x10000000;
const TO_ID_MASK: u32 = 0x0FF00000;
const FROM_ID_MASK: u32 = 0x000FF000;
const SEQ_NO_MASK: u32 = 0x00000FFF;
const PASSIVE_BIT_OFFSET: u32 = 28;
const TO_ID_OFFSET: u32 = 20;
const FROM_ID_OFFSET: u32 = 12;

const MAX_NUM_OF_WATCH_IDS: usize = 16;
static mut CAN_MESSENGER_ONCE: Once<CanMessenger<'static, MAX_NUM_OF_WATCH_IDS>> = Once::new();
pub fn get_can_messenger() -> &'static mut CanMessenger<'static, MAX_NUM_OF_WATCH_IDS> {
    unsafe { CAN_MESSENGER_ONCE.get_mut_or_fail() }
}

pub fn init_can_messenger(
    can_id: u8,
    core: &mut cortex_m::Peripherals,
    baud_rate: u32,
    can_rx_id: u32,
    can_tx_id: u32,
) {
    let messenger =
        CanMessenger::new(can2040::initialize_cbus(core, baud_rate, can_rx_id, can_tx_id), can_id);
    unsafe { CAN_MESSENGER_ONCE.init_once_with_variable(messenger) }
}

pub struct CanMessenger<'a, const N: usize> {
    can: Can2040,
    can_id: u8,
    seq_no: u16,
    watched_ids: Mutex<RefCell<FnvIndexSet<u32, N>>>,
    frames: Mutex<RefCell<FnvIndexMap<u32, CanFrame, N>>>,
    default_queue: Option<&'a mut MessageQueueWrapper<CanFrame>>,
}

impl<'a, const N: usize> CanMessenger<'a, N> {
    pub fn new(can: Can2040, can_id: u8) -> Self {
        Self {
            can,
            can_id,
            seq_no: 0,
            watched_ids: Mutex::new(RefCell::new(FnvIndexSet::new())),
            frames: Mutex::new(RefCell::new(FnvIndexMap::new())),
            default_queue: None,
        }
    }

    pub fn set_default_queue(&mut self, queue: &'a mut MessageQueueWrapper<CanFrame>) {
        self.default_queue = Some(queue);
    }

    pub fn add_watched_id(&self, id: u32) {
        critical_section::with(|cs| {
            self.watched_ids.borrow(cs).borrow_mut().insert(id).ok();
        });
    }

    pub fn receive(&mut self, frame: CanFrame) {
        let id = if let Id::Extended(eid) = frame.id() {
            eid.as_raw()
        } else {
            return;
        };

        critical_section::with(|cs| {
            if self.watched_ids.borrow(cs).borrow().contains(&id) {
                self.frames.borrow(cs).borrow_mut().insert(id, frame).ok();
            } else if let Some(queue) = self.default_queue.as_mut() {
                // TODO(zephyr): 确认一下cs1里面又调用cs2是不是会出问题？
                queue.enqueue(frame);
            }
        });
    }

    pub async fn receive_task(&mut self) {
        loop {
            match <Can2040 as embedded_can::nb::Can>::receive(&mut self.can) {
                Ok(frame) => {
                    self.receive(frame);
                    continue;
                }
                Err(nb::Error::WouldBlock) => {}
                Err(err) => {
                    defmt::error!("CAN receive error: {:?}", Debug2Format(&err));
                }
            }
            Delay::new(10.micros()).await;
        }
    }

    pub async fn wait_for_message_with_eid(&self, extended_id: u32) -> CanFrame {
        self.add_watched_id(extended_id);
        CanFrameFuture { receiver: self, id: extended_id }.await
    }

    // TODO(zephyr): 想一想这里是不是需要critical section保护。
    pub fn send_to<U: Serialize>(&mut self, to: u8, msg: U) -> Result<u32, AtomiError> {
        self.seq_no = self.seq_no.wrapping_add(1);
        let id_raw = PASSIVE_BIT
            | ((to as u32) << TO_ID_OFFSET)
            | ((self.can_id as u32) << FROM_ID_OFFSET)
            | (self.seq_no as u32 & SEQ_NO_MASK);
        self.send_raw(id_raw, msg)?;
        let resp_id = ((self.can_id as u32) << TO_ID_OFFSET)
            | ((to as u32) << FROM_ID_OFFSET)
            | (self.seq_no as u32 & SEQ_NO_MASK);
        Ok(resp_id)
    }

    pub fn send_raw<U: Serialize>(&mut self, id_raw: u32, msg: U) -> Result<(), AtomiError> {
        let data = postcard::to_allocvec::<U>(&msg).map_err(|_| AtomiError::UartInvalidInput)?;
        let id = Extended(ExtendedId::new(id_raw).ok_or(AtomiError::CanExtendedIdError)?);
        let frame = CanFrame::new(id, data.as_slice()).ok_or(AtomiError::CanFrameError)?;
        <Can2040 as embedded_can::blocking::Can>::transmit(&mut self.can, &frame)
            .map_err(|_| AtomiError::CanTransmitError)?;
        Ok(())
    }
}

pub struct CanFrameFuture<'a, const N: usize> {
    receiver: &'a CanMessenger<'a, N>,
    id: u32,
}

impl<'a, const N: usize> Future for CanFrameFuture<'a, N> {
    type Output = CanFrame;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let frame = critical_section::with(|cs| {
            self.receiver.frames.borrow(cs).borrow_mut().remove(&self.id)
        });

        if let Some(f) = frame {
            Poll::Ready(f)
        } else {
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

pub fn parse_id(id: Id) -> Result<(u8, u8, u8, u16), AtomiError> {
    if let Id::Extended(eid) = id {
        let raw_id = eid.as_raw();
        let passive_bit = ((raw_id >> PASSIVE_BIT_OFFSET) & 0x1) as u8;
        let to_id = ((raw_id >> TO_ID_OFFSET) & TO_ID_MASK) as u8;
        let from_id = ((raw_id >> FROM_ID_OFFSET) & FROM_ID_MASK) as u8;
        let seq_no = (raw_id & SEQ_NO_MASK) as u16;
        Ok((passive_bit, to_id, from_id, seq_no))
    } else {
        Err(AtomiError::CanExtendedIdError)
    }
}
pub fn gen_response_id(passive_bit: u8, to_id: u8, from_id: u8, seq_no: u16) -> u32 {
    (passive_bit as u32) << PASSIVE_BIT_OFFSET
        | (to_id as u32) << TO_ID_OFFSET
        | (from_id as u32) << FROM_ID_OFFSET
        | (seq_no as u32)
}

pub fn parse_frame<T: for<'a> Deserialize<'a>>(frame: CanFrame) -> Result<(u32, T), AtomiError> {
    let (passive_bit, to_id, from_id, seq_no) = parse_id(frame.id())?;
    let resp_id = gen_response_id(1 - passive_bit, from_id, to_id, seq_no);

    let data = frame.data()[..frame.dlc()].to_vec();
    let t = postcard::from_bytes::<T>(&data).map_err(|_| AtomiError::CanFrameError)?;

    Ok((resp_id, t))
}
