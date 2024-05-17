mod lidar;
mod motor_control;
mod odometry;
mod pose_graph;
mod tcp_server;
mod utils;

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use lidar::LidarEngine;
use motor_control::MotorControlRequest;
use pose_graph::PoseGraph;
use tcp_server::Client;
use tokio::sync::mpsc;
use utils::init_serialport;

use tokio::io::AsyncReadExt;
use tokio::net::{TcpListener, TcpStream};
use tokio_serial::SerialPort;

use crate::tcp_server::{CarToClient, ClientToCar};

#[tokio::main]
async fn main() {
    // state
    let pose_graph: Arc<Mutex<PoseGraph>> = Arc::new(Mutex::new(PoseGraph::new()));
    let motor_position = Arc::new(Mutex::new(0));
    let servo_us = Arc::new(Mutex::new(1500u16));
    let mut arduino_up = false; // if it is not up, do not send messages to it yet
    let (tx, mut rx) = mpsc::channel::<MotorControlRequest>(32);
    let tx = Arc::new(tx);

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

    let tcp_server_tx = tx.clone();
    let tcp_server_pose_graph = pose_graph.clone();
    // spawn the tcp listener on another thread
    tokio::spawn(async move {
        let listener = TcpListener::bind("0.0.0.0:49925").await.unwrap();
        loop {
            let (stream, addr) = listener.accept().await.unwrap();
            let client_tx = tcp_server_tx.clone();
            let pose_graph_client_thread = tcp_server_pose_graph.clone();

            // give each client its own green thread
            tokio::spawn(async move {
                let mut client = Client::new(stream);

                loop {
                    // handle client loop
                    if let Ok(packets) = client.poll().await {
                        println!("read {} packets", packets.len());
                        for packet in packets {
                            println!("received packet {:?}", packet);
                            match packet {
                                ClientToCar::GetCurrentPose => {
                                    // cry about it
                                }
                                ClientToCar::GetMostRecentLidarScan => {
                                    let scan = {
                                        let graph = pose_graph_client_thread.lock().unwrap();
                                        if let Some(node) = graph.nodes.last() {
                                            Some(node.scan.clone())
                                        } else {
                                            None
                                        }
                                    };
                                    if let Some(scan) = scan {
                                        CarToClient::LidarScan { scan: &scan }
                                            .write(&mut client.stream)
                                            .await
                                            .unwrap();
                                    }
                                }
                                ClientToCar::SetServoPosition { microseconds } => {
                                    client_tx
                                        .try_send(MotorControlRequest::SetServoPosition {
                                            microseconds,
                                        })
                                        .unwrap();
                                }
                                ClientToCar::SetMotorOutput(output) => {
                                    client_tx
                                        .try_send(MotorControlRequest::SetMotorOutput(output))
                                        .unwrap();
                                }
                            }
                        }
                    }
                }
            });
        }
    });

    // spawn the motor control thread
    let thread_motor_position = motor_position.clone();
    let thread_servo_us = servo_us.clone();
    tokio::spawn(async move {
        let mut arduino_port = init_serialport("/dev/ttyACM0");

        loop {
            if arduino_up {
                if let Ok(request) = rx.try_recv() {
                    println!("sending request: {:?}", request);
                    if let MotorControlRequest::SetServoPosition { microseconds } = request {
                        *thread_servo_us.lock().unwrap() = microseconds;
                    }
                    request.write(&mut arduino_port).await.unwrap();
                }
            }
            // 5 byte packet sent over and over again: 0b10101010, i32::to_le_bytes()
            if arduino_port.bytes_to_read().unwrap() >= 5 {
                arduino_up = true;
                let mut buffer = [0; 5];
                arduino_port.read_exact(&mut buffer).await.unwrap();
                // the first byte of the packet, think of it as a flag
                if buffer[0] == 0b10101010 {
                    // this could *technically* return an invalid motor position if the alignment of the packet is off. This could only reasonably happen if the arduino somehow starts at a position where the first byte is 0b10101010, which is not a normal starting configuration
                    let position = i32::from_le_bytes([buffer[1], buffer[2], buffer[3], buffer[4]]);
                    *thread_motor_position.lock().unwrap() = position;
                } else {
                    // alignment is off of the 5 byte packet
                    arduino_port.read_exact(&mut [0; 1]).await.unwrap();
                }
            }
        }
    });

    tx.send(MotorControlRequest::SetServoPosition { microseconds: 1450 })
        .await
        .unwrap();

    loop {}
}
