mod lidar;
mod motor_control;
use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream},
    thread,
    time::{Duration, Instant},
};

use lidar::LidarRequest;
use motor_control::{MotorControlRequest, MotorControlResponse};

use crate::lidar::LidarResponse;

fn main() {
    let listener = TcpListener::bind("0.0.0.0:49925").unwrap();
    listener
        .set_nonblocking(true)
        .expect("Cannot set non-blocking");

    let mut arduino_port = serialport::new("/dev/ttyACM0", 115200)
        .timeout(Duration::from_micros(1000))
        .open()
        .unwrap_or_else(|err| {
            eprintln!("Failed to open serial port: {}", err);
            println!("Here are the available ports:");
            let ports = serialport::available_ports().expect("No ports found!");
            for p in ports {
                println!("{}", p.port_name);
            }
            std::process::exit(1);
        });
    let mut lidar_port = serialport::new("/dev/ttyAMA0", 115200)
        .timeout(Duration::from_micros(1000))
        .open()
        .unwrap_or_else(|err| {
            eprintln!("Failed to open serial port: {}", err);
            println!("Here are the available ports:");
            let ports = serialport::available_ports().expect("No ports found!");
            for p in ports {
                println!("{}", p.port_name);
            }
            std::process::exit(1);
        });
    // thread::sleep(Duration::from_millis(1500));

    // loop {
    //     // get an input u16 from the user
    //     let mut input = String::new();
    //     println!("Enter a position for the servo motor (500-2500): ");
    //     std::io::stdin().read_line(&mut input).unwrap();
    //     let position: u16 = input.trim().parse().unwrap();
    //     if position < 1070 || position > 1820 {
    //         println!("Invalid input. Please enter a number between 1070 and 1820.");
    //         continue;
    //     }
    //     MotorControlRequest::SetServoPosition {
    //         microseconds: position,
    //     }
    //     .write(&mut arduino_port)
    //     .unwrap();
    // }

    let mut time = Instant::now();

    start_scan(&mut lidar_port);

    loop {
        if lidar_port.bytes_to_read().unwrap() >= 132 {
            let mut buf = [0; 132];
            lidar_port.read_exact(&mut buf).unwrap();
            println!("read 132 bytes in {:?}", time.elapsed());
            time = Instant::now();
        }
        if let Ok((stream, _)) = listener.accept() {
            thread::spawn(|| {
                handle_client(stream);
            });
        }
    }
}

fn handle_client(mut stream: TcpStream) {
    println!("Client connected: {:?}", stream.peer_addr().unwrap());
    loop {
        match echo_byte(&mut stream) {
            Ok(_) => {}
            Err(e) => match e.kind() {
                std::io::ErrorKind::UnexpectedEof | std::io::ErrorKind::ConnectionAborted => {
                    println!("Client disconnected: {:?}", stream.peer_addr().unwrap());
                    return;
                }
                _ => {
                    eprintln!("Error: {:?}", e);
                    return;
                }
            },
        }
    }
}

fn echo_byte(stream: &mut TcpStream) -> Result<(), std::io::Error> {
    let mut byte = [0];
    stream.read_exact(&mut byte)?;
    stream.write(&byte)?;
    Ok(())
}

fn set_position_and_wait(port: &mut Box<dyn serialport::SerialPort>, position: i32) {
    MotorControlRequest::SetMotorPosition { clicks: position }
        .write(port)
        .unwrap();
    thread::sleep(Duration::from_millis(150));
    loop {
        MotorControlRequest::GetMotorPosition.write(port).unwrap();
        while port.bytes_to_read().unwrap() < 4 {
            thread::sleep(Duration::from_millis(10));
        }
        let response = MotorControlResponse::read(port).unwrap();
        match response {
            MotorControlResponse::MotorPosition { clicks } => {
                if (clicks - position).abs() < 5 {
                    break;
                }
            }
        }
        thread::sleep(Duration::from_millis(150));
    }
}

fn set_servo_position_and_wait(port: &mut Box<dyn serialport::SerialPort>, position: u16) {
    MotorControlRequest::SetServoPosition {
        microseconds: position,
    }
    .write(port)
    .unwrap();
    thread::sleep(Duration::from_millis(1500));
}

fn start_scan(port: &mut Box<dyn serialport::SerialPort>) {
    println!("Initializing Lidar");
    LidarRequest::Stop.write(port).unwrap();
    // wait
    std::thread::sleep(Duration::from_millis(800));
    // clear buffer
    port.clear(serialport::ClearBuffer::Input).unwrap();
    port.write(&[0xa5, 0x79]).unwrap();
    std::thread::sleep(Duration::from_millis(800));
    let mut buf = [0; 22];
    port.read_exact(&mut buf).unwrap();
    assert_eq!(
        buf,
        [
            0xa5, 0x5a, 0x0f, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x61, 0x00, 0x00, 0x00, 0xa0,
            0x00, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x28, 0x1d
        ]
    );
    port.write(&[0xa5, 0x82, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x21])
        .unwrap();
    std::thread::sleep(Duration::from_millis(800));
    let mut buf = [0; 7];
    port.read_exact(&mut buf).unwrap();
    assert_eq!(buf, [0xa5, 0x5a, 0x84, 0x00, 0x00, 0x40, 0x84]);
    println!("Lidar started");
}
