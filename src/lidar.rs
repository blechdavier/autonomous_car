/*

 // Commands without payload and response
#define SL_LIDAR_CMD_STOP                   0x25
#define SL_LIDAR_CMD_SCAN                   0x20
#define SL_LIDAR_CMD_FORCE_SCAN             0x21
#define SL_LIDAR_CMD_RESET                  0x40

// Commands with payload but no response
#define SL_LIDAR_CMD_NEW_BAUDRATE_CONFIRM   0x90 //added in fw 1.30

// Commands without payload but have response
#define SL_LIDAR_CMD_GET_DEVICE_INFO        0x50
#define SL_LIDAR_CMD_GET_DEVICE_HEALTH      0x52

#define SL_LIDAR_CMD_GET_SAMPLERATE         0x59 //added in fw 1.17

#define SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL    0xA8


// Commands with payload and have response
#define SL_LIDAR_CMD_EXPRESS_SCAN           0x82 //added in fw 1.17
#define SL_LIDAR_CMD_HQ_SCAN                0x83 //added in fw 1.24
#define SL_LIDAR_CMD_GET_LIDAR_CONF         0x84 //added in fw 1.24
#define SL_LIDAR_CMD_SET_LIDAR_CONF         0x85 //added in fw 1.24
//add for A2 to set RPLIDAR motor pwm when using accessory board
#define SL_LIDAR_CMD_SET_MOTOR_PWM          0xF0
#define SL_LIDAR_CMD_GET_ACC_BOARD_FLAG     0xFF
 */

use std::time::{Duration, Instant};

pub struct LidarEngine {
    pub port: Box<dyn serialport::SerialPort>,
    pub scan_packets: Vec<ScanPacket>,
    pub scans: Vec<LidarScan>,
}

impl LidarEngine {
    pub fn new(port: Box<dyn serialport::SerialPort>) -> Self {
        let mut engine = Self {
            port,
            scan_packets: Vec::new(),
            scans: Vec::new(),
        };
        engine.init();
        engine
    }

    fn init(&mut self) {
        println!("Initializing Lidar");
        let tries = 5; // TODO constant
        for i in 0..tries {
            LidarRequest::Stop.write(&mut self.port).unwrap();
            // wait
            std::thread::sleep(Duration::from_millis(800));
            // clear buffer
            self.port.clear(serialport::ClearBuffer::Input).unwrap();
            self.port
                .write(&[0xa5, 0x82, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x21])
                .unwrap();
            std::thread::sleep(Duration::from_millis(800));
            let mut buf = [0; 7];
            self.port.read_exact(&mut buf).unwrap();
            if buf == [0xa5, 0x5a, 0x84, 0x00, 0x00, 0x40, 0x84] {
                println!("Lidar initialized");
                break;
            } else {
                println!(
                    "Lidar initialization failed, retrying {} more times",
                    tries - i - 1
                );
            }
        }
    }

    /// See if there are new scan packets and process them accordingly in order to optionally get a new scan
    pub fn poll(&mut self) -> Option<&LidarScan> {
        if self.port.bytes_to_read().unwrap() >= 132 {
            let scan_count = self.scans.len();
            let mut buffer = [0; 132];
            self.port.read_exact(&mut buffer).unwrap();
            self.scan_packets.push(ScanPacket::from_buffer(&buffer));
            if self.scan_packets.len() > 1 {
                for i in 0..32 {
                    let mut dist_q2 = [0; 3];

                    let combined_x3 =
                        self.scan_packets[self.scan_packets.len() - 2].ultra_cabins[i];

                    // unpack
                    let dist_major1 = combined_x3 & 0xFFF;
                    let mut dist_predict1 = ((combined_x3 as i32) << 10) >> 22;
                    let mut dist_predict2 = (combined_x3 as i32) >> 22;

                    let dist_major2 = if i == 31 {
                        &self.scan_packets[self.scan_packets.len() - 1].ultra_cabins[0] & 0xFFF
                    } else {
                        &self.scan_packets[self.scan_packets.len() - 2].ultra_cabins[i + 1] & 0xFFF
                    };

                    let mut scale_level1 = 0;
                    let mut scale_level2 = 0;

                    let dist_major1 = varbitscale_decode(dist_major1, &mut scale_level1);
                    let dist_major2 = varbitscale_decode(dist_major2, &mut scale_level2);

                    let mut dist_base1 = dist_major1;
                    let dist_base2 = dist_major2;

                    if dist_major1 == 0 && dist_major2 != 0 {
                        dist_base1 = dist_major2;
                        scale_level1 = scale_level2;
                    }

                    dist_q2[0] = dist_major1 << 2;
                    if dist_predict1 as u32 == 0xFFFFFE00 || dist_predict1 == 0x1FF {
                        dist_q2[1] = 0
                    } else {
                        dist_predict1 <<= scale_level1;
                        dist_q2[1] = ((dist_base1 as i32 + dist_predict1) << 2) as u32;
                    }

                    if dist_predict2 as u32 == 0xFFFFFE00 || dist_predict2 == 0x1FF {
                        dist_q2[2] = 0
                    } else {
                        dist_predict2 <<= scale_level2;
                        dist_q2[2] = ((dist_base2 as i32 + dist_predict2) << 2) as u32;
                    }

                    let start_angle_q6 =
                        self.scan_packets[self.scan_packets.len() - 2].start_angle_q6 as i32;
                    let angle_diff_q6 = (self.scan_packets[self.scan_packets.len() - 1]
                        .start_angle_q6 as i32
                        - start_angle_q6 as i32)
                        .rem_euclid(360 * 64);

                    for j in 0..3 {
                        let point = LidarPoint {
                            angle_q6: (start_angle_q6 as u16
                                + (angle_diff_q6 as f64 * (i as f64 / 32.0 + j as f64 / 96.0))
                                    as u16)
                                % (360 * 64),
                            distance_q0: dist_q2[j],
                            index: j as u8,
                        };

                        // reject 0 distance points, they represent points which are either too far or too close to be detected
                        if point.distance_q0 != 0 {
                            self.add_point(point);
                        }
                    }
                }
            }
            if self.scans.len() >= 2 && scan_count != self.scans.len() {
                return Some(&self.scans[self.scans.len() - 2]);
            }
        }
        None
    }

    fn add_point(&mut self, point: LidarPoint) {
        if self.scans.is_empty() {
            self.scans.push(LidarScan { points: Vec::new() });
        }
        let current_scan = self.scans.last_mut().unwrap();
        if current_scan.points.is_empty() {
            current_scan.points.push(point);
        } else {
            // wrap once the scan is complete
            let last_point = current_scan.points.last().unwrap();
            if last_point.angle_q6 > point.angle_q6 {
                self.scans.push(LidarScan {
                    points: vec![point],
                });
            } else {
                current_scan.points.push(point);
            }
        }
    }

    pub fn get_most_recent_scan(&self) -> Option<&LidarScan> {
        if self.scans.len() >= 2 {
            Some(&self.scans[self.scans.len() - 2])
        } else {
            None
        }
    }
}

#[derive(Debug)]
pub struct LidarPoint {
    pub angle_q6: u16,
    pub distance_q0: u32,
    pub index: u8,
}

impl LidarPoint {
    pub fn get_angle_degrees(&self) -> f32 {
        self.angle_q6 as f32 / 64.0
    }
    pub fn get_angle_rad(&self) -> f32 {
        self.angle_q6 as f32 * std::f32::consts::PI / 180.0 / 64.0
    }
}

pub struct ScanPacket {
    pub timestamp: Instant,
    pub start_bit: bool,
    pub start_angle_q6: u16,
    pub ultra_cabins: [u32; 32],
}

impl ScanPacket {
    fn from_buffer(bytes: &[u8; 132]) -> Self {
        let timestamp = Instant::now();
        let sync = (bytes[0] & 0xF0) | (bytes[1] >> 4);
        assert_eq!(sync, 0xa5);
        let checksum = (bytes[0] & 0xF) | (bytes[1] << 4);
        let mut check_checksum = 0;
        for i in 2..bytes.len() {
            check_checksum ^= bytes[i];
        }
        assert_eq!(checksum, check_checksum);
        let start_bit = bytes[3] & 0b1000_0000 != 0;
        let start_angle_q6 = u16::from_le_bytes([bytes[2], bytes[3] & 0b0111_1111]);

        if start_bit {
            println!("New Scan Started");
        }

        let mut ultra_cabins = Vec::with_capacity(32);
        for i in 0..32 {
            let offset = i * 4;
            let cabin = u32::from_le_bytes(bytes[(4 + offset)..(8 + offset)].try_into().unwrap());
            ultra_cabins.push(cabin);
        }
        ScanPacket {
            timestamp,
            start_bit,
            start_angle_q6,
            ultra_cabins: ultra_cabins.try_into().unwrap(),
        }
    }

    fn get_start_angle_radians(&self) -> f32 {
        self.start_angle_q6 as f32 * std::f32::consts::PI / 180.0 / 64.0
    }
}

const DEFAULT_SCAN: LidarScan = LidarScan { points: Vec::new() };

#[derive(Debug)]
pub struct LidarScan {
    pub points: Vec<LidarPoint>,
}

// communications
pub enum LidarRequest {
    Stop,
    Reset,
    GetDeviceInfo,
    GetDeviceHealth,
}

#[derive(Debug)]
pub enum LidarResponse {
    DeviceInfo {
        model: u8,
        firmware_minor: u8,
        firmware_major: u8,
        hardware: u8,
        serial: [u8; 16],
    },
}

impl LidarRequest {
    pub fn write(
        &self,
        port: &mut Box<dyn serialport::SerialPort>,
    ) -> Result<(), serialport::Error> {
        match self {
            LidarRequest::Stop => {
                port.write(&[0xa5, 0x25])?;
            }
            LidarRequest::Reset => {
                port.write(&[0xa5, 0x40])?;
            }
            LidarRequest::GetDeviceInfo => {
                port.write(&[0xa5, 0x50])?;
            }
            LidarRequest::GetDeviceHealth => {
                port.write(&[0xa5, 0x52])?;
            }
        }
        Ok(())
    }
}

impl LidarResponse {
    /// doesn't work yet. this might not be necessary
    pub fn read(port: &mut Box<dyn serialport::SerialPort>) -> Result<Self, serialport::Error> {
        let mut response_descriptor = [0; 7];
        port.read_exact(&mut response_descriptor)?;
        assert!(response_descriptor[0] == 0xa5);
        assert!(response_descriptor[1] == 0x5a);
        // 30bit length
        // 2bit send_mode
        // 1byte data_type

        let data_type = response_descriptor[6];
        let send_mode = response_descriptor[5] & 0b11;
        let length = ((response_descriptor[5] as u32) << 8) | response_descriptor[4] as u32;
        let mut buffer = [0; 20];
        port.read_exact(&mut buffer)?;
        Ok(LidarResponse::DeviceInfo {
            model: buffer[0],
            firmware_minor: buffer[1],
            firmware_major: buffer[2],
            hardware: buffer[3],
            serial: [
                buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], buffer[10],
                buffer[11], buffer[12], buffer[13], buffer[14], buffer[15], buffer[16], buffer[17],
                buffer[18], buffer[19],
            ], // TODO this is embarassing lol
        })
    }
}

fn varbitscale_decode(scaled: u32, scale_level: &mut u32) -> u32 {
    const VBS_SCALED_BASE: [u32; 5] = [3328, 1792, 1280, 512, 0];
    const VBS_SCALED_LVL: [u32; 5] = [4, 3, 2, 1, 0];
    const VBS_TARGET_BASE: [u32; 5] = [(0x1 << 14), (0x1 << 12), (0x1 << 11), (0x1 << 9), 0];

    for i in 0..VBS_SCALED_BASE.len() {
        let remain = scaled as i32 - VBS_SCALED_BASE[i] as i32;
        if remain >= 0 {
            *scale_level = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << *scale_level) as u32;
        }
    }
    0
}

#[test]
fn test_varbitscale_decode() {
    let mut scale_level = 0;
    assert_eq!(varbitscale_decode(1000, &mut scale_level), 1488);
    assert_eq!(scale_level, 1);
    scale_level = 0;
    assert_eq!(varbitscale_decode(2000, &mut scale_level), 5760);
    assert_eq!(scale_level, 3);
    scale_level = 0;
    assert_eq!(varbitscale_decode(1500, &mut scale_level), 2928);
    assert_eq!(scale_level, 2);
    scale_level = 0;
    assert_eq!(varbitscale_decode(15000, &mut scale_level), 203136);
    assert_eq!(scale_level, 4);
    scale_level = 0;
    assert_eq!(varbitscale_decode(0, &mut scale_level), 0);
    assert_eq!(scale_level, 0);
}
