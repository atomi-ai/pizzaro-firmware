use core::cell::UnsafeCell;

pub struct Once<T> {
    initialized: bool,
    value: UnsafeCell<Option<T>>,
}

impl<T> Default for Once<T> {
    fn default() -> Self {
        Self::new()
    }
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

    pub fn get_mut(&mut self) -> &mut T
    where
        T: Default,
    {
        self.init_once_with_function(T::default);
        unsafe { (*self.value.get()).as_mut().unwrap() }
    }

    pub fn get_mut_or_fail(&mut self) -> &mut T {
        if !self.initialized {
            panic!("Please initialize can-bus");
        }
        unsafe { (*self.value.get()).as_mut().unwrap() }
    }
}
