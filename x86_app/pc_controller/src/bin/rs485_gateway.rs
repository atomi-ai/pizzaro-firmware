use std::time::Duration;

use clap::Parser;
use generic::atomi_error::AtomiError;
use generic::atomi_proto::{AtomiProto, wrap_result_into_proto};
use serde::{Deserialize, Serialize};
use serialport::SerialPort;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};

use pc_controller::{find_serial_device, send_command};

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
#[command(next_line_help = true)]
struct Cli {
    #[arg(long)]
    #[arg(required_unless_present("rs485_probe"))]
    rs485_port: Option<String>,

    #[arg(long)]
    #[arg(required_unless_present("rs485_port"))]
    rs485_probe: Option<String>,

    #[arg(long)]
    service_port: u16,
}

#[derive(Serialize, Deserialize, Debug)]
struct CommandRequest {
    command: String,
}

#[derive(Serialize, Deserialize, Debug)]
struct CommandResponse {
    proto: AtomiProto,
}

async fn handle(socket: &mut TcpStream, port: &mut Box<dyn SerialPort>) -> Result<AtomiProto, AtomiError> {
    let mut buf = vec![0; 1024];
    println!("Wait for the request");
    let n = match socket.read(&mut buf).await {
        Ok(n) if n == 0 => Err(AtomiError::GWEmptyRequest), // socket closed
        Ok(n) => Ok(n),
        Err(_) => Err(AtomiError::GWReadSocket),
    }?;

    let req: CommandRequest = match serde_json::from_slice(&buf[..n]) {
        Ok(req) => Ok(req),
        Err(_) => Err(AtomiError::GWParseRequest),
    }?;
    println!("Get request: {:?}", req);

    send_command(req.command.as_str(), true, port)
}

#[tokio::main]
async fn main() -> rustyline::Result<()> {
    env_logger::init();

    let cli = Cli::parse();
    println!("cli = {:?}", cli);
    let port_name = match (cli.rs485_port, cli.rs485_probe) {
        (Some(port), _) => port,
        (_, Some(probe)) =>
            find_serial_device(probe.as_str())
                .expect(format!("Not found port with probe {}", probe).as_str()),
        _ => None.expect("No port or probe in your arguments"),
    };
    let mut port = serialport::new(port_name, 115_200)
        .timeout(Duration::from_secs(5))
        .open().expect("Failed to open port");

    let listener = TcpListener::bind(("0.0.0.0", cli.service_port)).await.unwrap();
    println!("Server listening on port {}", cli.service_port);

    loop {
        let (mut socket, _) = listener.accept().await.unwrap();
        loop {
            let response = wrap_result_into_proto(handle(&mut socket, &mut port).await);
            let response_json = serde_json::to_string(&response).unwrap();
            socket.write_all(response_json.as_bytes()).await.unwrap();
        }
    }
}
