#[derive(Debug)]
pub enum MotorControlRequest {
    GetMotorPosition,
    SetMotorPosition {
        clicks: i32,
    },
    /// Values between 500 and 2500 are full range of motion for the servo, so 1500 is center. In normal use, the car should never have to turn the servo this much.
    SetServoPosition {
        microseconds: u16,
    },
}

#[derive(Debug)]
pub enum MotorControlResponse {
    MotorPosition { clicks: i32 },
}

impl MotorControlResponse {
    pub fn read(
        port: &mut Box<dyn serialport::SerialPort>,
    ) -> Result<MotorControlResponse, serialport::Error> {
        let mut buffer = [0; 4];
        port.read_exact(&mut buffer)?;
        dbg!(buffer);
        Ok(MotorControlResponse::MotorPosition {
            clicks: i32::from_le_bytes(buffer),
        })
    }
}

impl MotorControlRequest {
    pub fn write(
        &self,
        port: &mut Box<dyn serialport::SerialPort>,
    ) -> Result<(), serialport::Error> {
        match self {
            MotorControlRequest::GetMotorPosition => {
                port.write(&[0x00])?;
            }
            MotorControlRequest::SetMotorPosition { clicks } => {
                port.write(&[0x01])?;
                let bytes = clicks.to_le_bytes();
                port.write(&bytes)?;
            }
            MotorControlRequest::SetServoPosition { microseconds } => {
                port.write(&[0x02])?;
                port.write(&microseconds.to_le_bytes())?;
            }
        }
        Ok(())
    }
}
