use core::cell::UnsafeCell;

pub struct Once<T> {
    initialized: bool,
    value: UnsafeCell<Option<T>>,
}

impl<T> Once<T> {
    pub const fn new() -> Once<T> {
        Once {
            initialized: false,
            value: UnsafeCell::new(None),
        }
    }

    // 新增一个方法来初始化值
    pub fn get_or_init<F>(&mut self, f: F) -> &mut T
        where
            F: FnOnce() -> T,
    {
        if !self.initialized {
            self.initialized = true;
            let value = f();
            unsafe { *self.value.get() = Some(value) };
        }

        unsafe { (*self.value.get()).as_mut().unwrap() }
    }

    // 修改 get_mut 方法
    pub fn get_mut(&mut self) -> &mut T
        where
            T: Default,
    {
        self.get_or_init(T::default)
    }
}
