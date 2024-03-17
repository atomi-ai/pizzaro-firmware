use serialport::{available_ports, SerialPortType};

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
