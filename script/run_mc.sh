DEFMT_LOG=pizzaro=info,can2040=debug,info cargo run --bin mc_main --release --config 'target."cfg(all(target_arch = \"arm\", target_os = \"none\"))".runner="probe-rs run --probe 2e8a:4005:E6635C469F598E2D --chip RP2040 --protocol swd --speed 1000"'

#cargo run --bin mc_main --release --config 'target."cfg(all(target_arch = \"arm\", target_os = \"none\"))".runner="probe-rs run --probe 2e8a:4005:E6635C469F7D192E --chip RP2040 --protocol swd --speed 1000"'
