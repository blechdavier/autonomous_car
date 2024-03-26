use std::{
    io::{Read, Write},
    net::TcpStream,
};

use crate::lidar::{LidarEngine, LidarScan};

pub fn poll_client(
    stream: &mut TcpStream,
    lidar_engine: &mut LidarEngine,
) -> Result<(), std::io::Error> {
    let mut packet_id = [0];
    match stream.read_exact(&mut packet_id) {
        Ok(_) => {
            let packet_id = packet_id[0];
            match packet_id {
                0 => {
                    CarToClient::CurrentPose {
                        x: 0.0,
                        y: 0.0,
                        theta: 0.0,
                    }
                    .write(stream)?;
                }
                1 => {
                    if let Some(scan) = lidar_engine.get_most_recent_scan() {
                        CarToClient::LidarScan { scan }.write(stream)?;
                    } else {
                        CarToClient::LidarScan {
                            scan: &LidarScan { points: Vec::new() },
                        }
                        .write(stream)?;
                    }
                }
                _ => panic!("Invalid packet id: {}", packet_id),
            }
        }
        Err(e) => {
            if e.kind() == std::io::ErrorKind::WouldBlock {
                return Ok(());
            }
            return Err(e);
        }
    }
    Ok(())
}

pub enum ClientToCar {
    GetCurrentPose,
    GetMostRecentLidarScan,
}

impl ClientToCar {
    pub fn read(stream: &mut TcpStream) -> Result<Self, std::io::Error> {
        let mut byte = [0];
        stream.read_exact(&mut byte)?;
        match byte[0] {
            0 => Ok(ClientToCar::GetCurrentPose),
            1 => Ok(ClientToCar::GetMostRecentLidarScan),
            _ => panic!("Invalid byte: {}", byte[0]),
        }
    }

    pub fn get_length_from_packet_id(id: u8) -> Result<usize, std::io::Error> {
        match id {
            0 => Ok(1),
            1 => Ok(1),
            _ => Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Invalid packet id",
            )),
        }
    }
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
                stream.write(&x.to_ne_bytes())?;
                stream.write(&y.to_ne_bytes())?;
                stream.write(&theta.to_ne_bytes())?;
            }
            CarToClient::LidarScan { scan } => {
                stream.write(&[1])?;
                stream.write(&(scan.points.len() as u32).to_ne_bytes())?;
                for point in &scan.points {
                    stream.write(&point.angle_q6.to_ne_bytes())?;
                    stream.write(&point.distance_q0.to_ne_bytes())?;
                    stream.write(&point.index.to_ne_bytes())?;
                }
            }
        }
        Ok(())
    }
}
