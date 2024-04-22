use core::cell::UnsafeCell;

pub struct Once<T> {
    initialized: bool,
    value: UnsafeCell<Option<T>>,
}

impl<T> Once<T> {
    pub const fn new() -> Once<T> {
        Once { initialized: false, value: UnsafeCell::new(None) }
    }

    pub fn init_once_with_variable(&mut self, value: T) {
        if !self.initialized {
            self.initialized = true;
            unsafe { *self.value.get() = Some(value) };
        }
    }

    pub fn init_once_with_function<F>(&mut self, f: F)
    where
        F: FnOnce() -> T,
    {
        if !self.initialized {
            let value = f();
            self.init_once_with_variable(value);
        }
    }

    // TODO(zephyr): Assume we don't need the function below.
    fn get_or_init<F>(&mut self, f: F) -> &mut T
    where
        F: FnOnce() -> T,
    {
        self.init_once_with_function(f);
        unsafe { (*self.value.get()).as_mut().unwrap() }
    }

    // TODO(zephyr): 将下面两个函数合并，不要在get_mut()里面初始化变量。
    pub fn get_mut(&mut self) -> &mut T
    where
        T: Default,
    {
        self.get_or_init(T::default)
    }

    pub fn get_mut_or_fail(&mut self) -> &mut T {
        if !self.initialized {
            panic!("Please initialize can-bus");
        }
        unsafe { (*self.value.get()).as_mut().unwrap() }
    }
}
