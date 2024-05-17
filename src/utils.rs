use std::time::Duration;

use tokio_serial::{SerialPortBuilderExt, SerialStream};

pub fn init_serialport(port: &str) -> SerialStream {
    tokio_serial::new(port, 115200)
        .timeout(Duration::from_micros(1000))
        .open_native_async()
        .unwrap_or_else(|err| {
            eprintln!("Failed to open serial port {}: {}", port, err);
            println!("Here are the available ports:");
            let ports = tokio_serial::available_ports().expect("No ports found!");
            for p in ports {
                println!("{}", p.port_name);
            }
            panic!();
        })
}
