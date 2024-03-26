use std::time::Duration;

pub fn init_serialport(port: &str) -> Box<dyn serialport::SerialPort> {
    serialport::new(port, 115200)
        .timeout(Duration::from_micros(1000))
        .open()
        .unwrap_or_else(|err| {
            eprintln!("Failed to open serial port {}: {}", port, err);
            println!("Here are the available ports:");
            let ports = serialport::available_ports().expect("No ports found!");
            for p in ports {
                println!("{}", p.port_name);
            }
            std::process::exit(1);
        })
}
