use defmt::debug;

// TODO(zephyr): Please rename the file to stationary_detector.
#[derive(Debug)]
struct CircularBuffer<T, const N: usize> {
    buffer: [T; N],
    head: usize,
    tail: usize,
    count: usize,
}

impl<T, const N: usize> CircularBuffer<T, N> {
    fn new(default_value: T) -> Self
        where
            T: Copy,
    {
        CircularBuffer {
            buffer: [default_value; N],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    fn push(&mut self, item: T) {
        self.buffer[self.head] = item;
        self.head = (self.head + 1) % N;
        if self.count == N {
            self.tail = (self.tail + 1) % N;
        } else {
            self.count += 1;
        }
    }

    fn iter(&self) -> impl Iterator<Item = &T> {
        (0..self.count).map(move |i| &self.buffer[(self.tail + i) % N])
    }
}

#[derive(Debug)]
pub(crate) struct StationaryDetector<const WINDOW_SIZE: usize> {
    window: CircularBuffer<(i64, f64), WINDOW_SIZE>,
    time_threshold: i64,
    distance_threshold: f64,
}

impl<const WINDOW_SIZE: usize> StationaryDetector<WINDOW_SIZE> {
    pub(crate) fn new(time_threshold: i64, distance_threshold: f64) -> Self {
        StationaryDetector {
            window: CircularBuffer::new((0, 0.0)),
            time_threshold,
            distance_threshold,
        }
    }

    pub(crate) fn is_stationary(&mut self, ts: i64, pos: f64) -> bool {
        debug!("StationaryDetector::is_stationary(), ts: {}, pos: {}", ts, pos);
        self.window.push((ts, pos));

        if self.window.count < WINDOW_SIZE {
            return false;
        }

        let sum: f64 = self.window.iter().map(|(_, p)| p).sum();
        let mean = sum / (self.window.count as f64);

        let stationary = self.window.iter().all(|(_, p)| abs(*p - mean) <= self.distance_threshold);
        let elapsed_time = self.window.iter().last().unwrap().0 - self.window.iter().next().unwrap().0;

        stationary && elapsed_time >= self.time_threshold
    }

    pub(crate) fn reset_stationary(&mut self) {
        self.window = CircularBuffer::new((0, 0.0))
    }
}

fn abs(value: f64) -> f64 {
    if value < 0.0 {
        -value
    } else {
        value
    }
}