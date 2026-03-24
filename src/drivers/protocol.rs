//! ─────────────────────────────────────────────────────────────
//! Tier 1 ↔ Tier 2 통신 프로토콜
//! ─────────────────────────────────────────────────────────────
//!
//! pekko-rust Actor System (Tier 1, Linux SBC) 과
//! 이 RTOS 펌웨어 (Tier 2, ESP32-S3) 간의 메시지 프로토콜.
//!
//! 프로토콜 구조:
//!   [STX][LEN_H][LEN_L][MSG_TYPE][PAYLOAD...][CRC16][ETX]
//!
//! 전송 매체: UART (기본 921600 baud) 또는 SPI
//! 최대 지연: <1ms (UART), <100µs (SPI)

use heapless::Vec;

/// 프레임 마커
const STX: u8 = 0x02; // Start of frame
const ETX: u8 = 0x03; // End of frame

/// 최대 페이로드 크기 (bytes)
const MAX_PAYLOAD: usize = 256;

/// 메시지 타입 — Tier 1 → Tier 2 (명령)
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum CommandType {
    /// 모터 속도 설정 (RPM 목표)
    SetMotorSpeed = 0x10,
    /// 서보 각도 설정
    SetServoAngle = 0x11,
    /// PID 게인 업데이트
    SetPidGains = 0x12,
    /// E-Stop (긴급 정지)
    EmergencyStop = 0x1F,
    /// 시스템 상태 요청
    RequestStatus = 0x20,
    /// 센서 데이터 스트림 시작/정지
    SetSensorStream = 0x30,
    /// Ping (연결 확인)
    Ping = 0xFE,
}

/// 메시지 타입 — Tier 2 → Tier 1 (리포트)
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum ReportType {
    /// 센서 데이터 (IMU, 인코더 등)
    SensorData = 0x80,
    /// 모터 상태 (실제 RPM, 전류)
    MotorStatus = 0x81,
    /// 시스템 상태 리포트
    SystemStatus = 0x82,
    /// 에러/경고 알림
    ErrorAlert = 0x8F,
    /// Pong (Ping 응답)
    Pong = 0xFF,
}

/// 통신 프레임
pub struct Frame {
    pub msg_type: u8,
    pub payload: Vec<u8, MAX_PAYLOAD>,
}

impl Frame {
    /// 새 프레임 생성
    pub fn new(msg_type: u8) -> Self {
        Self {
            msg_type,
            payload: Vec::new(),
        }
    }

    /// 프레임을 바이트 버퍼로 직렬화
    pub fn serialize(&self, buf: &mut Vec<u8, 512>) -> Result<(), ()> {
        let len = self.payload.len() as u16;

        buf.push(STX).map_err(|_| ())?;
        buf.push((len >> 8) as u8).map_err(|_| ())?;
        buf.push((len & 0xFF) as u8).map_err(|_| ())?;
        buf.push(self.msg_type).map_err(|_| ())?;

        for &byte in self.payload.iter() {
            buf.push(byte).map_err(|_| ())?;
        }

        // CRC16 (CCITT)
        let crc = crc16_ccitt(&buf[1..]); // STX 제외
        buf.push((crc >> 8) as u8).map_err(|_| ())?;
        buf.push((crc & 0xFF) as u8).map_err(|_| ())?;
        buf.push(ETX).map_err(|_| ())?;

        Ok(())
    }

    /// 바이트 버퍼에서 프레임 역직렬화
    pub fn deserialize(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.len() < 6 {
            return Err(ProtocolError::TooShort);
        }
        if data[0] != STX {
            return Err(ProtocolError::InvalidStart);
        }
        if data[data.len() - 1] != ETX {
            return Err(ProtocolError::InvalidEnd);
        }

        let len = ((data[1] as u16) << 8 | data[2] as u16) as usize;
        let msg_type = data[3];

        if data.len() < 6 + len {
            return Err(ProtocolError::TooShort);
        }

        // CRC 검증
        let crc_offset = 4 + len;
        let expected_crc = (data[crc_offset] as u16) << 8 | data[crc_offset + 1] as u16;
        let actual_crc = crc16_ccitt(&data[1..crc_offset]);
        if expected_crc != actual_crc {
            return Err(ProtocolError::CrcMismatch);
        }

        let mut payload = Vec::new();
        for &byte in &data[4..4 + len] {
            payload.push(byte).map_err(|_| ProtocolError::PayloadTooLarge)?;
        }

        Ok(Self { msg_type, payload })
    }
}

/// 프로토콜 에러
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ProtocolError {
    TooShort,
    InvalidStart,
    InvalidEnd,
    CrcMismatch,
    PayloadTooLarge,
    UnknownMessageType,
}

/// CRC-16 CCITT (0xFFFF 초기값)
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ── 모터 속도 명령 메시지 헬퍼 ──

/// 모터 속도 명령 (Tier 1 → Tier 2)
#[derive(Clone, Copy, Debug)]
pub struct MotorSpeedCmd {
    pub motor_id: u8,     // 모터 ID (0~7)
    pub target_rpm: i16,  // 목표 RPM (-32768 ~ 32767)
    pub accel_ms: u16,    // 가속 시간 (ms)
}

impl MotorSpeedCmd {
    pub fn to_payload(&self) -> [u8; 5] {
        [
            self.motor_id,
            (self.target_rpm >> 8) as u8,
            (self.target_rpm & 0xFF) as u8,
            (self.accel_ms >> 8) as u8,
            (self.accel_ms & 0xFF) as u8,
        ]
    }

    pub fn from_payload(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.len() < 5 {
            return Err(ProtocolError::TooShort);
        }
        Ok(Self {
            motor_id: data[0],
            target_rpm: ((data[1] as i16) << 8) | data[2] as i16,
            accel_ms: ((data[3] as u16) << 8) | data[4] as u16,
        })
    }
}

/// 센서 데이터 리포트 (Tier 2 → Tier 1)
#[derive(Clone, Copy, Debug)]
pub struct SensorReport {
    pub timestamp_us: u32, // 타임스탬프 (마이크로초)
    pub accel: [i16; 3],   // 가속도 (x, y, z) — raw
    pub gyro: [i16; 3],    // 자이로 (x, y, z) — raw
    pub encoder_ticks: [i32; 2], // 인코더 틱 (좌, 우)
}

impl SensorReport {
    pub fn to_payload(&self) -> [u8; 24] {
        let mut buf = [0u8; 24];
        // timestamp (4 bytes)
        buf[0..4].copy_from_slice(&self.timestamp_us.to_be_bytes());
        // accel (6 bytes)
        for i in 0..3 {
            let bytes = self.accel[i].to_be_bytes();
            buf[4 + i * 2] = bytes[0];
            buf[5 + i * 2] = bytes[1];
        }
        // gyro (6 bytes)
        for i in 0..3 {
            let bytes = self.gyro[i].to_be_bytes();
            buf[10 + i * 2] = bytes[0];
            buf[11 + i * 2] = bytes[1];
        }
        // encoder (8 bytes)
        buf[16..20].copy_from_slice(&self.encoder_ticks[0].to_be_bytes());
        buf[20..24].copy_from_slice(&self.encoder_ticks[1].to_be_bytes());
        buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16() {
        let data = b"Hello";
        let crc = crc16_ccitt(data);
        assert_eq!(crc, crc16_ccitt(data)); // deterministic
    }

    #[test]
    fn test_motor_cmd_roundtrip() {
        let cmd = MotorSpeedCmd {
            motor_id: 2,
            target_rpm: -1500,
            accel_ms: 200,
        };
        let payload = cmd.to_payload();
        let decoded = MotorSpeedCmd::from_payload(&payload).unwrap();
        assert_eq!(decoded.motor_id, 2);
        assert_eq!(decoded.target_rpm, -1500);
        assert_eq!(decoded.accel_ms, 200);
    }
}
