# Pizzaro Project

## Run
Start MMD / HPD / MC:
```shell
cargo run --release --package pizzaro --bin mmd_main -- --probe 2e8a:000c:<mmd_serial>
cargo run --release --package pizzaro --bin hpd_main -- --probe 2e8a:000c:<hpd_serial>
cargo run --release --package pizzaro --bin mc_main -- --probe 2e8a:000c:<mc_serial>
```

Run pc_controller to connect MC:
```shell
cargo run --bin pc_controller -- --probe 16c0:27dd
```

You can use command "[list_ports](https://github.com/serialport/serialport-rs/blob/main/examples/list_ports.rs])" to list all usb serial ports connected with the machine:
```shell
$ list_ports 
Found 5 ports:
  /dev/ttyACM1
    Type: USB
    VID:2e8a PID:000c
     Serial Number: 65CD5C1714224E53
      Manufacturer: Raspberry Pi
           Product: Picoprobe (CMSIS-DAP)
  /dev/ttyACM3
    Type: USB
    VID:16c0 PID:27dd
     Serial Number: TEST
      Manufacturer: Fake company
           Product: Serial port
  /dev/ttyACM0
    Type: USB
    VID:2e8a PID:000c
     Serial Number: 4150335631373503
      Manufacturer: Raspberry Pi
           Product: Picoprobe (CMSIS-DAP)
  /dev/ttyACM2
    Type: USB
    VID:2e8a PID:000c
     Serial Number: E4632448A72B5931
      Manufacturer: Raspberry Pi
           Product: Picoprobe (CMSIS-DAP)
  /dev/ttyS0
    Type: Unknown
```

## Commit Criteria
Please make sure your code passed the criteria below for each of your commits:
```shell
cargo build --release && cargo build --release --examples
python integration/run_all_tests.py
python integration/run_smoke_test.py
```
