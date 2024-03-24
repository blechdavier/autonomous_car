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
