use std::io::{Read, Write};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio_serial::SerialStream;

use crate::{
    lidar::{LidarEngine, LidarScan},
    motor_control::MotorControlRequest,
};

pub struct Client {
    pub stream: TcpStream,
    buffer: Vec<u8>,
}

impl Client {
    pub fn new(stream: TcpStream) -> Self {
        Self {
            stream,
            buffer: Vec::new(),
        }
    }
    pub async fn poll(&mut self) -> Result<Vec<ClientToCar>, std::io::Error> {
        let mut buf = [0; 64];

        match self.stream.read(&mut buf).await {
            Ok(n) => {
                if n > 0 {
                    println!("read {} bytes", n);
                }
                self.buffer.extend_from_slice(&buf[..n]);
            }
            Err(e) => {
                return Err(e);
            }
        }

        let mut packets = Vec::new();
        loop {
            match self.buffer.get(0) {
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

                    if self.buffer.len() >= length {
                        let buf: Vec<u8> = self.buffer.drain(0..length).collect();
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

                        packets.push(request);
                    } else {
                        return Ok(packets);
                    }
                }
                None => {
                    return Ok(packets);
                }
            };
        }
    }
}

#[derive(Debug)]
pub enum ClientToCar {
    GetCurrentPose,
    GetMostRecentLidarScan,
    SetServoPosition { microseconds: u16 },
    SetMotorOutput(i16),
}

#[derive(Debug)]
pub enum CarToClient<'a> {
    CurrentPose { x: f32, y: f32, theta: f32 },
    LidarScan { scan: &'a LidarScan },
}

impl CarToClient<'_> {
    pub async fn write(&self, stream: &mut TcpStream) -> Result<(), std::io::Error> {
        // println!("sending {:?}", &self);
        match self {
            CarToClient::CurrentPose { x, y, theta } => {
                stream.write(&[0]).await?;
                stream.write(&x.to_le_bytes()).await?;
                stream.write(&y.to_le_bytes()).await?;
                stream.write(&theta.to_le_bytes()).await?;
            }
            CarToClient::LidarScan { scan } => {
                stream.write(&[1]).await?;
                stream
                    .write(&(scan.points.len() as u32).to_le_bytes())
                    .await?;
                for point in &scan.points {
                    stream.write(&point.angle_q6.to_le_bytes()).await?;
                    stream.write(&point.distance_q0.to_le_bytes()).await?;
                    stream.write(&point.index.to_le_bytes()).await?;
                }
            }
        }
        Ok(())
    }
}
