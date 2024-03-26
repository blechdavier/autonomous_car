mod lidar;
mod motor_control;
mod tcp_server;
mod utils;

use std::{net::TcpListener, time::Duration};

use lidar::LidarEngine;
use tcp_server::poll_client;
use utils::init_serialport;

fn main() {
    let listener = TcpListener::bind("0.0.0.0:49925").unwrap();
    listener
        .set_nonblocking(true)
        .expect("Cannot set non-blocking");

    let mut _arduino_port = init_serialport("/dev/ttyACM0");
    let mut tcp_connections = Vec::new();

    let mut lidar_engine = LidarEngine::new(init_serialport("/dev/ttyAMA0"));

    // loop control
    let mut timestamp = std::time::Instant::now(); // used for keeping track of loop times
    let mut loop_count: u64 = 0; // how many loops have been run

    loop {
        // handle motor control
        {}

        // read lidar data
        if let Some(scan) = lidar_engine.poll() {
            println!(
                "Lidar scan added with start angle {}deg, end angle {}deg, {} points",
                scan.points[0].get_angle_degrees(),
                scan.points[scan.points.len() - 1].get_angle_degrees(),
                scan.points.len()
            );
        }

        // handle tcp connections
        if loop_count % 20 == 0 {
            if let Ok((stream, addr)) = listener.accept() {
                println!("client {} connected", addr);
                tcp_connections.push(stream);
            }

            let mut i = 0;
            if tcp_connections.len() > 0 {
                println!("handling {} clients", tcp_connections.len());
            }
            while i < tcp_connections.len() {
                match poll_client(&mut tcp_connections[i], &mut lidar_engine) {
                    Ok(_) => {
                        i += 1;
                    }
                    Err(_) => {
                        tcp_connections.remove(i);
                    }
                }
            }
        }

        if timestamp.elapsed() > Duration::from_millis(5) {
            eprintln!("Loop time overrun: {}us", timestamp.elapsed().as_micros());
        } else {
            std::thread::sleep(std::time::Duration::from_micros(5000) - timestamp.elapsed());
        }
        loop_count += 1;
        timestamp = std::time::Instant::now();
    }
}
