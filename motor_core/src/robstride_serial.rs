use crate::bus::{CanBus, CanFrame};
use crate::error::{MotorError, Result};
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::Mutex;
use std::time::{Duration, Instant};

/// Frame format (Lingzu/RobStride proprietary AT-frame protocol):
///   TX/RX: b"AT" + addr(4B big-endian) + dlc(1B) + data(dlc bytes) + b"\r\n"
///   addr encoding: (can_id_29bit << 3) | 0x04

const FRAME_HEADER: &[u8; 2] = b"AT";
const FRAME_TRAILER: &[u8; 2] = b"\r\n";
const ADDR_FLAG: u32 = 0x04;

struct Inner {
    port: Box<dyn SerialPort>,
    rx_buf: VecDeque<u8>,
}

pub struct RobstrideSerialBus {
    inner: Mutex<Inner>,
}

impl RobstrideSerialBus {
    pub fn open(port: &str, baud: u32) -> Result<Self> {
        let port_obj = serialport::new(port, baud)
            .timeout(Duration::from_millis(2))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .flow_control(FlowControl::None)
            .open()
            .map_err(|e| MotorError::Io(format!("open serial port {port} failed: {e}")))?;
        Ok(Self {
            inner: Mutex::new(Inner {
                port: port_obj,
                rx_buf: VecDeque::with_capacity(1024),
            }),
        })
    }

    fn encode_tx(frame: CanFrame) -> Result<Vec<u8>> {
        if frame.dlc > 8 {
            return Err(MotorError::InvalidArgument(format!(
                "invalid DLC {}, expected <= 8",
                frame.dlc
            )));
        }

        // AT(2) + addr(4) + dlc(1) + data(dlc) + \r\n(2)
        let addr = (frame.arbitration_id << 3) | ADDR_FLAG;
        let dlc = frame.dlc as usize;
        let mut out = Vec::with_capacity(7 + dlc + FRAME_TRAILER.len());
        out.extend_from_slice(FRAME_HEADER);
        out.extend_from_slice(&addr.to_be_bytes());
        out.push(frame.dlc);
        out.extend_from_slice(&frame.data[..dlc]);
        out.extend_from_slice(FRAME_TRAILER);
        Ok(out)
    }

    fn try_parse_rx(buf: &mut VecDeque<u8>) -> Option<CanFrame> {
        loop {
            // Find "AT" header
            loop {
                if buf.len() < 2 {
                    return None;
                }
                if buf[0] == FRAME_HEADER[0] && buf[1] == FRAME_HEADER[1] {
                    break;
                }
                let _ = buf.pop_front();
            }

            // Need at least: AT(2) + addr(4) + dlc(1) = 7, then dlc bytes + \r\n
            if buf.len() < 7 {
                return None;
            }
            let dlc = buf[6] as usize;
            if dlc > 8 {
                // Bad frame, skip the "AT" and try again.
                let _ = buf.pop_front();
                let _ = buf.pop_front();
                continue;
            }
            let frame_len = 7 + dlc + 2; // header + addr + dlc + data + trailer
            if buf.len() < frame_len {
                return None;
            }

            // Verify trailer
            if buf[frame_len - 2] != FRAME_TRAILER[0] || buf[frame_len - 1] != FRAME_TRAILER[1] {
                // Bad frame, skip the "AT" and try again.
                let _ = buf.pop_front();
                let _ = buf.pop_front();
                continue;
            }

            // Parse addr -> CAN ID
            let addr = ((buf[2] as u32) << 24)
                | ((buf[3] as u32) << 16)
                | ((buf[4] as u32) << 8)
                | (buf[5] as u32);
            let arbitration_id = addr >> 3;

            // Parse data
            let mut data = [0u8; 8];
            for i in 0..dlc {
                data[i] = buf[7 + i];
            }

            // Consume frame from buffer
            for _ in 0..frame_len {
                let _ = buf.pop_front();
            }

            return Some(CanFrame {
                arbitration_id,
                data,
                dlc: dlc as u8,
                is_extended: true,
                is_rx: true,
            });
        }
    }
}

impl CanBus for RobstrideSerialBus {
    fn send(&self, frame: CanFrame) -> Result<()> {
        let raw = Self::encode_tx(frame)?;
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("robstride-serial lock poisoned".to_string()))?;
        inner
            .port
            .write_all(&raw)
            .map_err(|e| MotorError::Io(format!("robstride-serial write failed: {e}")))?;
        Ok(())
    }

    fn recv(&self, timeout: Duration) -> Result<Option<CanFrame>> {
        let deadline = Instant::now()
            .checked_add(timeout)
            .unwrap_or_else(|| Instant::now() + Duration::from_secs(3600));
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("robstride-serial lock poisoned".to_string()))?;

        loop {
            if let Some(frame) = Self::try_parse_rx(&mut inner.rx_buf) {
                return Ok(Some(frame));
            }

            let mut tmp = [0u8; 256];
            match inner.port.read(&mut tmp) {
                Ok(n) if n > 0 => {
                    inner.rx_buf.extend(tmp[..n].iter().copied());
                }
                Ok(_) => {}
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(MotorError::Io(format!("robstride-serial read failed: {e}"))),
            }

            if Instant::now() >= deadline {
                return Ok(None);
            }
        }
    }

    fn shutdown(&self) -> Result<()> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("robstride-serial lock poisoned".to_string()))?;
        inner
            .port
            .flush()
            .map_err(|e| MotorError::Io(format!("robstride-serial flush failed: {e}")))?;
        Ok(())
    }
}
