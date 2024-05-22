use generic::atomi_error::AtomiError;
use generic::atomi_proto::AtomiProto;
use generic::command_to_proto::parse_protocol;
use serialport::{available_ports, SerialPort, SerialPortType};

pub fn find_serial_device(probe: &str) -> Option<String> {
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

/// Parses a command into a request, sends the request to the backend,
/// and waits for the response.
///
/// # Arguments
///
/// * `command` - A string slice that holds the command to be parsed and sent.
///
/// # Returns
///
/// * `Result<ResponseType, ErrorType>` - Returns the response from the backend or an error.
pub fn send_command(command: &str, with_len: bool, port: &mut Box<dyn SerialPort>) -> Result<AtomiProto, AtomiError> {
    let msg = parse_protocol(&command);
    println!("Got msg: {:?}", msg);
    if matches!(msg, AtomiProto::Unknown) {
        return Err(AtomiError::UnaccepableCommand);
    }
    let data = postcard::to_vec::<AtomiProto, 8>(&msg).map_err(|_| AtomiError::GWIncorrectRequest)?;

    if with_len {
        let l = data.len() as u8;
        println!("Send data len: {}", l);
        port.write(&[l]).map_err(|_| AtomiError::GWSendingDataLen)?;
    }
    println!("Line: {}, protocol: {:?}, data = {:?}", command, msg, data);
    port.write(&data).map_err(|_| AtomiError::GWSendingData)?;

    let mut buf: Vec<u8> = vec![0u8; 64];
    let len = port.read(buf.as_mut_slice()).map_err(|_| AtomiError::GWRecvResponse)?;
    println!("Got response data: {:?}, len: {}", &buf[..len], len);
    let resp_data = if with_len {
        assert_eq!((len - 1) as u8, buf[0].into());
        &buf[1..len]
    } else {
        &buf[..len]
    };
    let resp = postcard::from_bytes::<AtomiProto>(resp_data).map_err(|_| AtomiError::GWParseResponse)?;
    println!("Got response: ({len}) {:?}, msg: {:?}", &buf[..len], resp);
    port.flush().map_err(|_| AtomiError::GWFlushPort)?;

    Ok(resp)
}
