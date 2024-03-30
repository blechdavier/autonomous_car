use tokio::io::AsyncWriteExt;
use tokio_serial::SerialStream;

#[derive(Debug)]
pub enum MotorControlRequest {
    SetMotorPosition {
        clicks: i32,
    },
    /// Values between 500 and 2500 are full range of motion for the servo, so 1500 is center. In normal use, the car should never have to turn the servo this much.
    SetServoPosition {
        microseconds: u16,
    },
    /// Values between -255 and 255 are allowed. Negative values are reverse, positive values are forward.
    SetMotorOutput(i16),
}

impl MotorControlRequest {
    pub async fn write(&self, port: &mut SerialStream) -> Result<(), tokio_serial::Error> {
        match self {
            MotorControlRequest::SetMotorPosition { clicks } => {
                port.write(&[0x01]).await?;
                let bytes = clicks.to_le_bytes();
                port.write(&bytes).await?;
            }
            MotorControlRequest::SetServoPosition { microseconds } => {
                port.write(&[0x02]).await?;
                port.write(&microseconds.to_le_bytes()).await?;
            }
            MotorControlRequest::SetMotorOutput(output) => {
                port.write(&[0x03]).await?;
                port.write(&output.to_le_bytes()).await?;
            }
        }
        Ok(())
    }
}
