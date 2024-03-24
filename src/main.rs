mod motor_control;
use std::{thread, time::Duration};

use motor_control::{MotorControlRequest, MotorControlResponse};

fn main() {
    let mut port = serialport::new("/dev/ttyACM0", 115200)
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
    thread::sleep(Duration::from_millis(1500));
    loop {
        // get an input u16 from the user
        let mut input = String::new();
        println!("Enter a position for the servo motor (500-2500): ");
        std::io::stdin().read_line(&mut input).unwrap();
        let position: u16 = input.trim().parse().unwrap();
        if position < 1070 || position > 1820 {
            println!("Invalid input. Please enter a number between 1070 and 1820.");
            continue;
        }
        MotorControlRequest::SetServoPosition {
            microseconds: position,
        }
        .write(&mut port)
        .unwrap();
    }
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
