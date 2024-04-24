DEFMT_LOG=info cargo run --bin dtu_main --release --config 'target."cfg(all(target_arch = \"arm\", target_os = \"none\"))".runner="probe-rs run --probe 2e8a:4005:E6635C469F10842D --chip RP2040 --protocol swd --speed 1000"'

#cargo run --bin dtu_main --release --config 'target."cfg(all(target_arch = \"arm\", target_os = \"none\"))".runner="probe-rs run --probe 2e8a:4005:E6635C469F10842D --chip RP2040 --protocol swd --speed 1000"'
