#cargo run --bin mmd_main --release --config 'target."cfg(all(target_arch = \"arm\", target_os = \"none\"))".runner="probe-rs run --probe 2e8a:000c:454741505A8D477A --chip RP2040 --protocol swd --speed 1000"'

cargo run --bin mmd_main --release --config 'target."cfg(all(target_arch = \"arm\", target_os = \"none\"))".runner="probe-rs run --probe 2e8a:4005:E6632C85933A5A27 --chip RP2040 --protocol swd --speed 1000"'
