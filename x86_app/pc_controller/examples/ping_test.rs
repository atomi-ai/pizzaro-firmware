use std::io::Write;
use std::time::Duration;
use clap::Parser;
use serialport::{available_ports, SerialPort, SerialPortType};
use generic::atomi_proto::{AtomiProto, HpdCommand, McCommand, MmdCommand};

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
#[command(next_line_help = true)]
struct CommandLineArgs {
    #[arg(long)]
    probe: String,
}

struct Cli {
    port: Box<dyn SerialPort>,
}

impl Cli {
    fn send_and_expect(&mut self, req: AtomiProto, exp: AtomiProto) {
        let data = postcard::to_vec::<AtomiProto, 8>(&req).unwrap();
        self.port.write(&data).expect("Send request error");
        let mut buf: Vec<u8> = vec![0; 8];
        let len = self.port.read(buf.as_mut_slice()).expect("Recv response error");
        let resp = postcard::from_bytes::<AtomiProto>(&buf[..len]).unwrap();
        if resp != exp {
            panic!("Unexpected response: {:?}", resp);
        }
    }
}

fn find_serial_device(probe: &str) -> Option<String> {
    let probe_parts: Vec<&str> = probe.split(':').collect();
    if probe_parts.len() != 2 {
        return None;
    }
    let vid = u16::from_str_radix(probe_parts[0], 16).ok()?;
    let pid = u16::from_str_radix(probe_parts[1], 16).ok()?;

    if let Ok(ports) = available_ports() {
        for p in ports {
            if let SerialPortType::UsbPort(info) = p.port_type {
                if info.vid == vid && info.pid == pid {
                    return Some(p.port_name);
                }
            }
        }
    }

    None
}

fn main() {
    env_logger::init();
    let args = CommandLineArgs::parse();
    println!("args = {:?}", args);

    let port_name = find_serial_device(&args.probe).unwrap();
    println!("port_name = {}", port_name);
    let mut cli = Cli {
        port: serialport::new(port_name, 115_200)
            .timeout(Duration::from_secs(5))
            .open().expect("Failed to open port"),
    };

    cli.send_and_expect(AtomiProto::Mc(McCommand::McPing), AtomiProto::Mc(McCommand::McPong));
    cli.send_and_expect(AtomiProto::Mmd(MmdCommand::MmdPing), AtomiProto::Mmd(MmdCommand::MmdPong));
    cli.send_and_expect(AtomiProto::Hpd(HpdCommand::HpdPing), AtomiProto::Hpd(HpdCommand::HpdPong));

    println!("ping_test PASSED");
}