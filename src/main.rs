mod lidar;
mod motor_control;
mod odometry;
mod pose_graph;
mod tcp_server;
mod utils;

use std::{
    fs,
    net::TcpListener,
    sync::{Arc, Mutex},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use lidar::{LidarEngine, LidarScan};
use motor_control::MotorControlRequest;
use pose_graph::PoseGraph;
use tcp_server::{poll_client, Client};
use tokio::sync::mpsc;
use utils::init_serialport;

use tokio::io::AsyncReadExt;
use tokio_serial::SerialPort;

#[tokio::main]
async fn main() {
    // state
    let pose_graph: Arc<Mutex<PoseGraph>> = Arc::new(Mutex::new(PoseGraph::new()));
    let motor_position = Arc::new(Mutex::new(0));
    let servo_us = Arc::new(Mutex::new(1500u16));
    let (tx, mut rx) = mpsc::channel::<MotorControlRequest>(32);

    let pose_graph_lidar_thread = pose_graph.clone();

    // spawn the lidar engine on one thread
    tokio::spawn(async move {
        let mut lidar_engine = LidarEngine::new(init_serialport("/dev/ttyAMA0")).await;
        loop {
            if let Some(scan) = lidar_engine.poll().await {
                pose_graph_lidar_thread
                    .lock()
                    .unwrap()
                    .add_node(scan.clone());
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
    let thread_servo_us = Arc::new(Mutex::new(1500u16));
    tokio::spawn(async move {
        let mut arduino_port = init_serialport("/dev/ttyACM0");

        loop {
            if let Ok(request) = rx.try_recv() {
                println!("sending request: {:?}", request);
                if let MotorControlRequest::SetServoPosition { microseconds } = request {
                    *thread_servo_us.lock().unwrap() = microseconds;
                }
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

    tx.send(MotorControlRequest::SetMotorOutput(255))
        .await
        .unwrap();

    loop {
        // handle motor control
        {
            let clicks = motor_position.lock().unwrap().clone();
            let servo_angle = servo_us.lock().unwrap().clone();
        }

        {
            // println!("there are {} scans", lidar_scans.lock().unwrap().len());
        }
    }
}
