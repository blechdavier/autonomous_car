mod motor_control;
use std::{thread, time::Duration};

use motor_control::{MotorControlRequest, MotorControlResponse};

fn main() {
    let mut port = serialport::new("/dev/tty.usbmodem1201", 115200)
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
    thread::sleep(Duration::from_millis(500));
    // set_position_and_wait(&mut port, 1000);
    // set_servo_position_and_wait(&mut port, 1200);
    // set_position_and_wait(&mut port, 500);
    // set_position_and_wait(&mut port, 1000);
    // set_servo_position_and_wait(&mut port, 1800);
    // set_position_and_wait(&mut port, 1500);
    // set_position_and_wait(&mut port, 1000);
    // set_servo_position_and_wait(&mut port, 1500);
    set_servo_position_and_wait(&mut port, 1430);
    set_servo_position_and_wait(&mut port, 1630);
    set_position_and_wait(&mut port, 5000);
    set_position_and_wait(&mut port, -400);

    set_servo_position_and_wait(&mut port, 1230);
    set_position_and_wait(&mut port, 5000);
    set_position_and_wait(&mut port, -400);
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
