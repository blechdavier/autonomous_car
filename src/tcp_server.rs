use std::{
    io::{Read, Write},
    net::TcpStream,
};

use tokio_serial::SerialStream;

use crate::{
    lidar::{LidarEngine, LidarScan},
    motor_control::MotorControlRequest,
};

pub struct Client {
    stream: TcpStream,
    buffer: Vec<u8>,
}

impl Client {
    pub fn new(stream: TcpStream) -> Self {
        Self {
            stream,
            buffer: Vec::new(),
        }
    }
}

pub async fn poll_client(
    client: &mut Client,
    arduino_port: &mut SerialStream,
) -> Result<(), std::io::Error> {
    let mut buf = [0; 64];

    match client.stream.read(&mut buf) {
        Ok(n) => {
            println!("read {} bytes", n);
            client.buffer.extend_from_slice(&buf[..n]);
        }
        Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
            println!("would block");
        }
        Err(e) => {
            return Err(e);
        }
    }

    match client.buffer.get(0) {
        Some(&id) => {
            let length = match id {
                0 => 1,
                1 => 1,
                2 => 3,
                3 => 3,
                _ => {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "Invalid packet id",
                    ))
                }
            };

            if client.buffer.len() >= length {
                let buf: Vec<u8> = client.buffer.drain(0..length).collect();
                let request = match id {
                    0 => ClientToCar::GetCurrentPose,
                    1 => ClientToCar::GetMostRecentLidarScan,
                    2 => {
                        let microseconds = u16::from_le_bytes([buf[1], buf[2]]);
                        ClientToCar::SetServoPosition { microseconds }
                    }
                    3 => {
                        let output = i16::from_le_bytes([buf[1], buf[2]]);
                        ClientToCar::SetMotorOutput(output)
                    }
                    _ => {
                        return Err(std::io::Error::new(
                            std::io::ErrorKind::InvalidData,
                            "Invalid packet id",
                        ))
                    }
                };

                match request {
                    ClientToCar::GetCurrentPose => {
                        let response = CarToClient::CurrentPose {
                            x: 0.0,
                            y: 0.0,
                            theta: 0.0,
                        };
                        response.write(&mut client.stream)?;
                    }
                    ClientToCar::GetMostRecentLidarScan => {
                        let default_scan = LidarScan { points: Vec::new() };
                        // let scan = lidar_engine.get_most_recent_scan().unwrap_or(&default_scan);
                        let response = CarToClient::LidarScan {
                            scan: &default_scan,
                        };
                        response.write(&mut client.stream)?;
                    }
                    ClientToCar::SetServoPosition { microseconds } => {
                        MotorControlRequest::SetServoPosition { microseconds }
                            .write(arduino_port)
                            .await?;
                    }
                    ClientToCar::SetMotorOutput(output) => {
                        MotorControlRequest::SetMotorOutput(output)
                            .write(arduino_port)
                            .await?;
                    }
                };
            }
        }
        None => {}
    };
    Ok(())
}

pub enum ClientToCar {
    GetCurrentPose,
    GetMostRecentLidarScan,
    SetServoPosition { microseconds: u16 },
    SetMotorOutput(i16),
}

pub enum CarToClient<'a> {
    CurrentPose { x: f32, y: f32, theta: f32 },
    LidarScan { scan: &'a LidarScan },
}

impl CarToClient<'_> {
    pub fn write(&self, stream: &mut TcpStream) -> Result<(), std::io::Error> {
        match self {
            CarToClient::CurrentPose { x, y, theta } => {
                stream.write(&[0])?;
                stream.write(&x.to_le_bytes())?;
                stream.write(&y.to_le_bytes())?;
                stream.write(&theta.to_le_bytes())?;
            }
            CarToClient::LidarScan { scan } => {
                stream.write(&[1])?;
                stream.write(&(scan.points.len() as u32).to_le_bytes())?;
                for point in &scan.points {
                    stream.write(&point.angle_q6.to_le_bytes())?;
                    stream.write(&point.distance_q0.to_le_bytes())?;
                    stream.write(&point.index.to_le_bytes())?;
                }
            }
        }
        Ok(())
    }
}
