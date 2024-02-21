extern crate alloc;

use alloc::boxed::Box;
use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use critical_section::Mutex;
use defmt::{info, Formatter, Format};
use fugit::ExtU64;
use futures::task::noop_waker;
use crate::global_timer::Delay;
use heapless::spsc::Queue;

const TASK_CAPACITY: usize = 32;

pub struct Executor {
    id: u8,
    tasks: Mutex<RefCell<Queue<Pin<Box<dyn Future<Output = ()> + Send>>, TASK_CAPACITY>>>,
}

impl Format for Executor {
    fn format(&self, fmt: Formatter) {
        defmt::write!(fmt, "Executor({}): task_num = {}", self.id, self.get_tasks_len());
    }
}

impl Executor {
    pub fn new(id: u8) -> Self {
        Executor {
            id,
            tasks: Mutex::new(RefCell::new(Queue::new())),
        }
    }

    fn get_tasks_len(&self) -> usize {
        critical_section::with(|cs| {
            self.tasks.borrow(cs).borrow().len()
        })
    }

    fn pop_task(&self) -> Option<Pin<Box<dyn Future<Output = ()> + Send>>> {
        critical_section::with(|cs| {
            self.tasks.borrow(cs).borrow_mut().dequeue()
        })
    }


    fn push_task(&self, fut_pin: Pin<Box<dyn Future<Output = ()> + Send + 'static>>) {
        // let free_bytes = global_allocator::ALLOCATOR.free();
        // info!("Free bytes in heap: {}", free_bytes);
        // log_stack_usage();

        let _t = critical_section::with(|cs| {
            self.tasks.borrow(cs).borrow_mut().enqueue(fut_pin)
        });
    }

    pub fn spawn_internal(&self, fut: impl Future<Output = ()> + Send + 'static) {
        if self.get_tasks_len() < TASK_CAPACITY {
            self.push_task(Box::pin(fut));
            info!("Added one task, task num = {}", self.get_tasks_len());
        } else {
            info!("Error: Too many tasks, ignoring new task");
            // Handle error as appropriate for your application
        }
    }

    pub fn run_executor_internal(&self) {
        let waker = noop_waker();
        let mut context = Context::from_waker(&waker);

        while let Some(mut task) = self.pop_task() {
            match task.as_mut().poll(&mut context) {
                Poll::Ready(_) => {
                    info!("Task completed, task_num = {}", self.get_tasks_len());
                }
                Poll::Pending => {
                    self.push_task(task);
                }
            }
        }
    }
}

static mut GLOBAL_EXECUTOR: Option<Executor> = None;

pub async fn dump_executor_status() {
    loop {
        unsafe {
            info!("dump_executor_status() {}", GLOBAL_EXECUTOR.as_ref());
        }
        Delay::new(5.secs()).await;
        info!("dump_executor_status() 9: wait one round");
    }
}

// TODO(zephyr): 暂时还不能支持随时随地spawn一个新task。我之前的调查发现，在运行时，
// 我们spawn()的新task好像不会出现在running tasks里面。这个挺奇怪的，以后再调.
pub fn spawn_task(fut: impl Future<Output = ()> + Send + 'static) {
    unsafe {
        if GLOBAL_EXECUTOR.is_none() {
            info!("spawn() new executor");
            GLOBAL_EXECUTOR.replace(Executor::new(1));
        }
        GLOBAL_EXECUTOR.as_mut().unwrap().spawn_internal(fut);
    }
}

pub fn start_global_executor() {
    unsafe {
        if GLOBAL_EXECUTOR.is_none() {
            info!("run() new executor");
            GLOBAL_EXECUTOR.replace(Executor::new(2));
        }
        GLOBAL_EXECUTOR.as_mut().unwrap().run_executor_internal();
    }
}
