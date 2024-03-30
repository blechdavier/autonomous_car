mod lidar;
mod motor_control;
mod tcp_server;
mod utils;

use std::{
    net::TcpListener,
    ops::Deref,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use lidar::{LidarEngine, LidarScan};
use motor_control::MotorControlRequest;
use tcp_server::{poll_client, Client};
use tokio::sync::mpsc;
use utils::init_serialport;

use tokio::io::AsyncReadExt;
use tokio_serial::SerialPort;

#[tokio::main]
async fn main() {
    // state
    let lidar_scans: Arc<Mutex<Vec<LidarScan>>> = Arc::new(Mutex::new(Vec::new()));
    let motor_position = Arc::new(Mutex::new(0));
    let (tx, mut rx) = mpsc::channel::<MotorControlRequest>(32);

    let lidar_scans_thread = lidar_scans.clone();

    // spawn the lidar engine on one thread
    tokio::spawn(async move {
        let mut lidar_engine = LidarEngine::new(init_serialport("/dev/ttyAMA0")).await;
        loop {
            if let Some(scan) = lidar_engine.poll().await {
                lidar_scans_thread.lock().unwrap().push(scan.clone());
            }
        }
    });

    // spawn the tcp listener on another thread
    tokio::spawn(async move {
        let listener = TcpListener::bind("0.0.0.0:49925").unwrap();
        loop {
            let (stream, _) = listener.accept().unwrap();

            // give each client its own green thread
            tokio::spawn(async move {
                let mut client = Client::new(stream);

                loop {
                    // handle client loop
                }
            });
        }
    });

    // spawn the motor control thread
    let thread_motor_position = motor_position.clone();
    tokio::spawn(async move {
        let mut arduino_port = init_serialport("/dev/ttyACM0");

        loop {
            if let Ok(request) = rx.try_recv() {
                request.write(&mut arduino_port).await.unwrap();
            }
            // 5 byte packet sent over and over again: 0b10101010, i32::to_le_bytes()
            if arduino_port.bytes_to_read().unwrap() >= 5 {
                let mut buffer = [0; 5];
                arduino_port.read_exact(&mut buffer).await.unwrap();
                // the first byte of the packet, think of it as a flag
                if buffer[0] == 0b10101010 {
                    // this could *technically* return an invalid motor position if the alignment of the packet is off. This could only reasonably happen if the arduino somehow starts at a position where the first byte is 0b10101010, which is not a normal starting configuration
                    let position = i32::from_le_bytes([buffer[1], buffer[2], buffer[3], buffer[4]]);
                    dbg!(position);
                    *thread_motor_position.lock().unwrap() = position;
                } else {
                    // alignment is off of the 5 byte packet
                    arduino_port.read_exact(&mut [0; 1]).await.unwrap();
                }
            }
        }
    });

    loop {
        // handle motor control
        {
            let clicks = motor_position.lock().unwrap().clone();
            // println!("motor position: {}", clicks);
            if clicks <= 0 {
                tx.send(MotorControlRequest::SetMotorPosition { clicks: 1000 })
                    .await
                    .unwrap();
            } else if clicks >= 1000 {
                tx.send(MotorControlRequest::SetMotorPosition { clicks: 0 })
                    .await
                    .unwrap();
            }
        }

        {
            // println!("there are {} scans", lidar_scans.lock().unwrap().len());
        }
    }
}
