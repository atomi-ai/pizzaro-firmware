# pizzaro device No.1
cargo run --release --package high_power_motor_driver --config 'target."cfg(all(target_arch = \"arm\", target_os = \"none\"))".runner="probe-rs run --probe 2e8a:4005:E6635C469F2A432E,--chip RP2040 --protocol swd --speed 1000"'
