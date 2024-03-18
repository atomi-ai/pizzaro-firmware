use alloc::collections::VecDeque;
use core::cell::RefCell;

use critical_section::Mutex;

pub trait MessageQueueInterface<T> {
    fn enqueue(&mut self, message: T);
    fn dequeue(&mut self) -> Option<T>;
    fn clear(&mut self);
}

pub struct MessageQueue<T> {
    queue: VecDeque<T>,
}

impl<T> MessageQueue<T> {
    pub fn new() -> Self {
        MessageQueue {
            queue: VecDeque::new(),
        }
    }
}

impl<T> Default for MessageQueue<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> MessageQueueInterface<T> for MessageQueue<T> {
    fn enqueue(&mut self, message: T) {
        self.queue.push_back(message);
    }

    fn dequeue(&mut self) -> Option<T> {
        self.queue.pop_front()
    }

    fn clear(&mut self) {
        self.queue.clear()
    }
}

pub struct MessageQueueWrapper<T>(Mutex<RefCell<MessageQueue<T>>>);

impl<T> Default for MessageQueueWrapper<T> {
    fn default() -> Self {
        Self(Mutex::new(RefCell::new(MessageQueue::new())))
    }
}

impl<T> MessageQueueInterface<T> for MessageQueueWrapper<T> {
    fn enqueue(&mut self, message: T) {
        critical_section::with(|cs| self.0.borrow(cs).borrow_mut().enqueue(message))
    }

    fn dequeue(&mut self) -> Option<T> {
        critical_section::with(|cs| self.0.borrow(cs).borrow_mut().dequeue())
    }

    fn clear(&mut self) {
        critical_section::with(|cs| self.0.borrow(cs).borrow_mut().clear())
    }
}
