extern crate alloc;

use alloc::boxed::Box;
use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use crate::common::global_allocator::dump_memory_usage;
use critical_section::Mutex;
use defmt::{debug, info, warn, Format, Formatter};
use fugit::ExtU64;
use futures::task::noop_waker;
use heapless::spsc::Queue;

use crate::common::global_timer::Delay;

const TASK_CAPACITY: usize = 32;
type TaskFuture = Pin<Box<dyn Future<Output = ()> + Send>>;
type TaskQueue = Queue<TaskFuture, TASK_CAPACITY>;

pub struct Executor {
    id: u8,
    tasks: Mutex<RefCell<TaskQueue>>,
}

impl Format for Executor {
    fn format(&self, fmt: Formatter) {
        defmt::write!(fmt, "Executor({}): task_num = {}", self.id, self.get_tasks_len());
    }
}

impl Executor {
    pub fn new(id: u8) -> Self {
        Executor { id, tasks: Mutex::new(RefCell::new(Queue::new())) }
    }

    fn get_tasks_len(&self) -> usize {
        critical_section::with(|cs| self.tasks.borrow(cs).borrow().len())
    }

    fn pop_task(&self) -> Option<Pin<Box<dyn Future<Output = ()> + Send>>> {
        critical_section::with(|cs| self.tasks.borrow(cs).borrow_mut().dequeue())
    }

    fn push_task(&self, fut_pin: Pin<Box<dyn Future<Output = ()> + Send + 'static>>) {
        // let free_bytes = global_allocator::ALLOCATOR.free();
        // info!("Free bytes in heap: {}", free_bytes);
        // log_stack_usage();

        let _t = critical_section::with(|cs| self.tasks.borrow(cs).borrow_mut().enqueue(fut_pin));
    }

    pub fn spawn_internal(&self, fut: impl Future<Output = ()> + Send + 'static) {
        if self.get_tasks_len() < TASK_CAPACITY {
            self.push_task(Box::pin(fut));
            debug!("Added one task, task num = {}", self.get_tasks_len());
        } else {
            warn!("Error: Too many tasks, ignoring new task");
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
        dump_memory_usage();
        Delay::new(5.secs()).await;
        info!("dump_executor_status() 9: wait one round");
    }
}

// 目前的实现已经可以在spawned task里面再spawn task了。
pub fn spawn_task(fut: impl Future<Output = ()> + Send + 'static) {
    unsafe {
        if GLOBAL_EXECUTOR.is_none() {
            debug!("spawn() new executor");
            GLOBAL_EXECUTOR.replace(Executor::new(1));
        }
        GLOBAL_EXECUTOR.as_mut().unwrap().spawn_internal(fut);
    }
}

pub fn start_global_executor() {
    unsafe {
        if GLOBAL_EXECUTOR.is_none() {
            debug!("run() new executor");
            GLOBAL_EXECUTOR.replace(Executor::new(2));
        }
        GLOBAL_EXECUTOR.as_mut().unwrap().run_executor_internal();
    }
}
