extern crate alloc;

use alloc::boxed::Box;
use core::cell::UnsafeCell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use fugit::TimerDurationU64;
use fugit::TimerInstantU64;

pub type AtomiInstant = TimerInstantU64<1_000_000>;
pub type AtomiDuration = TimerDurationU64<1_000_000>;

pub trait AtomiTimer: Sync + Send {
    fn now(&self) -> AtomiInstant;
}

static mut GLOBAL_TIMER: UnsafeCell<Option<Box<dyn AtomiTimer>>> = UnsafeCell::new(None);

pub fn init_global_timer(timer: Box<dyn AtomiTimer>) {
    unsafe {
        *GLOBAL_TIMER.get() = Some(timer);
    }
}

pub fn now() -> AtomiInstant {
    unsafe {
        let t = GLOBAL_TIMER.get().as_ref();
        match t {
            Some(Some(timer)) => timer.as_ref().now(),
            _ => AtomiInstant::from_ticks(0),
        }
    }
}

pub trait AsyncDelay {
    ///
    /// The interface is the same as:
    /// ```
    ///    async fn delay(&mut self, duration: AtomiDuration);
    ///```
    /// Let's wait for [async-fn-in-trait](https://blog.rust-lang.org/inside-rust/2022/11/17/async-fn-in-trait-nightly.html) feature, and change it back.
    fn delay(&mut self, duration: AtomiDuration) -> impl Future<Output = ()> + Send;
}

pub struct Delay {
    start: AtomiInstant,
    duration: AtomiDuration,
}

impl Delay {
    pub fn new(duration: AtomiDuration) -> Self {
        Self {
            start: now(),
            duration,
        }
    }

    pub fn wait(&self) {
        loop {
            if let Some(duration) = now().checked_duration_since(self.start) {
                if duration >= self.duration {
                    break;
                }
            }
        }
    }
}

impl Future for Delay {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Self::Output> {
        if let Some(duration) = now().checked_duration_since(self.start) {
            if duration >= self.duration {
                return Poll::Ready(());
            }
        }
        Poll::Pending
    }
}

pub struct DelayCreator;

impl DelayCreator {
    pub fn new() -> Self {
        DelayCreator {}
    }
}

impl AsyncDelay for DelayCreator {
    async fn delay(&mut self, duration: AtomiDuration) {
        Delay::new(duration).await
    }
}
