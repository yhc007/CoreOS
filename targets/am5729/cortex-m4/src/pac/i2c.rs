//! ─────────────────────────────────────────────────────────────
//! AM5729 I2C 드라이버 (Cortex-M4 IPU1)
//! ─────────────────────────────────────────────────────────────
//!
//! AM5729의 I2C 컨트롤러는 OMAP I2C IP (HS I2C)입니다.
//! I2C1 (0x4807_0000)을 사용하여 IMU 센서와 통신합니다.
//!
//! 지원 센서:
//!   - MPU6050 (addr 0x68/0x69) — 6축 IMU
//!   - ICM42688-P (addr 0x68/0x69) — 6축 IMU (고성능)
//!   - BNO055 (addr 0x28/0x29) — 9축 IMU + 센서 퓨전
//!
//! I2C 클럭 설정:
//!   - 시스템 입력 클럭: 96 MHz (FUNCCLK from DPLL)
//!   - I2C 내부 클럭: 12 MHz (프리스케일러)
//!   - SCL: 400 kHz (Fast Mode)
//!
//! 참고: TRM SPRUHZ6 — Chapter 23: I2C (Inter-Integrated Circuit)
//!
//! ═══════════════════════════════════════════════════════════════
//!  레지스터 맵 (OMAP HS I2C — TRM Table 23-4)
//! ═══════════════════════════════════════════════════════════════
//!
//! Offset  Register           Description
//! 0x00    I2C_REVNB_LO       Revision (low)
//! 0x04    I2C_REVNB_HI       Revision (high)
//! 0x10    I2C_SYSC            System Configuration
//! 0x24    I2C_IRQSTATUS_RAW  IRQ Status (raw)
//! 0x28    I2C_IRQSTATUS      IRQ Status (masked + W1C)
//! 0x2C    I2C_IRQENABLE_SET  IRQ Enable Set
//! 0x30    I2C_IRQENABLE_CLR  IRQ Enable Clear
//! 0x48    I2C_BUF            Buffer Configuration
//! 0x4C    I2C_CNT            Data Counter
//! 0x50    I2C_DATA           Data Access
//! 0x54    I2C_CON            Configuration
//! 0x60    I2C_OA             Own Address
//! 0x64    I2C_SA             Slave Address
//! 0x68    I2C_PSC            Prescaler
//! 0x6C    I2C_SCLL           SCL Low Time
//! 0x70    I2C_SCLH           SCL High Time
//! 0x74    I2C_SYSTEST        System Test
//! 0x88    I2C_BUFSTAT        Buffer Status
//! 0x98    I2C_IRQSTATUS_RAW  (alias)
//! 0xA4    I2C_OA1            Own Address 1
//! 0xAC    I2C_ACTOA          Active Own Address

use crate::pac;

// ═══════════════════════════════════════════════════════
// 레지스터 오프셋
// ═══════════════════════════════════════════════════════

pub mod reg {
    pub const REVNB_LO: u32 = 0x00;
    pub const REVNB_HI: u32 = 0x04;
    pub const SYSC: u32 = 0x10;
    pub const IRQSTATUS_RAW: u32 = 0x24;
    pub const IRQSTATUS: u32 = 0x28;
    pub const IRQENABLE_SET: u32 = 0x2C;
    pub const IRQENABLE_CLR: u32 = 0x30;
    pub const WE: u32 = 0x34;         // Wakeup Enable
    pub const DMARXENABLE_SET: u32 = 0x38;
    pub const DMATXENABLE_SET: u32 = 0x3C;
    pub const DMARXENABLE_CLR: u32 = 0x40;
    pub const DMATXENABLE_CLR: u32 = 0x44;
    pub const BUF: u32 = 0x48;
    pub const CNT: u32 = 0x4C;
    pub const DATA: u32 = 0x50;
    pub const CON: u32 = 0x54;
    pub const OA: u32 = 0x60;
    pub const SA: u32 = 0x64;
    pub const PSC: u32 = 0x68;
    pub const SCLL: u32 = 0x6C;
    pub const SCLH: u32 = 0x70;
    pub const SYSTEST: u32 = 0x74;
    pub const BUFSTAT: u32 = 0x88;
}

// ═══════════════════════════════════════════════════════
// I2C_CON 비트 필드 (TRM 23.1.7.6)
// ═══════════════════════════════════════════════════════

pub mod con {
    /// I2C Enable (Module enable)
    pub const I2C_EN: u32 = 1 << 15;
    /// Master mode
    pub const MST: u32 = 1 << 10;
    /// Transmitter (1) / Receiver (0)
    pub const TRX: u32 = 1 << 9;
    /// Start condition (master mode)
    pub const STT: u32 = 1 << 0;
    /// Stop condition (master mode)
    pub const STP: u32 = 1 << 1;
    /// 10-bit addressing mode (0 = 7-bit)
    pub const XSA: u32 = 1 << 8;
}

// ═══════════════════════════════════════════════════════
// I2C_IRQSTATUS 비트 필드 (TRM 23.1.7.2)
// ═══════════════════════════════════════════════════════

pub mod irq {
    /// Arbitration Lost
    pub const AL: u32 = 1 << 0;
    /// No Acknowledgement
    pub const NACK: u32 = 1 << 1;
    /// Register Access Ready (bus free)
    pub const ARDY: u32 = 1 << 2;
    /// Receive Data Ready
    pub const RRDY: u32 = 1 << 3;
    /// Transmit Data Ready
    pub const XRDY: u32 = 1 << 4;
    /// General Call
    pub const GC: u32 = 1 << 5;
    /// Bus Busy
    pub const BB: u32 = 1 << 12;
    /// Bus Free
    pub const BF: u32 = 1 << 8;
    /// Addressed as Slave
    pub const AAS: u32 = 1 << 9;
    /// Transmit underflow
    pub const XUDF: u32 = 1 << 10;
    /// Receive overflow
    pub const ROVR: u32 = 1 << 11;

    /// All error flags
    pub const ALL_ERRORS: u32 = AL | NACK | XUDF | ROVR;
    /// All completion/data flags
    pub const ALL_STATUS: u32 = ARDY | RRDY | XRDY | BF;
}

// ═══════════════════════════════════════════════════════
// I2C_SYSC 비트 필드 (TRM 23.1.7.1)
// ═══════════════════════════════════════════════════════

pub mod sysc {
    /// Soft Reset
    pub const SRST: u32 = 1 << 1;
    /// Auto Idle Enable
    pub const AUTOIDLE: u32 = 1 << 0;
    /// Smart-idle mode
    pub const SMARTIDLE: u32 = 2 << 3;
    /// Clock Activity: both clocks maintained
    pub const CLKACTIVITY_BOTH: u32 = 3 << 8;
}

// ═══════════════════════════════════════════════════════
// I2C 에러 타입
// ═══════════════════════════════════════════════════════

#[derive(Copy, Clone, Debug, PartialEq, Eq, defmt::Format)]
pub enum I2cError {
    /// No ACK received from slave
    Nack,
    /// Bus arbitration lost
    ArbitrationLost,
    /// Bus busy (timeout waiting for bus free)
    BusBusy,
    /// Transfer timeout
    Timeout,
    /// TX underflow
    TxUnderflow,
    /// RX overflow
    RxOverflow,
}

// ═══════════════════════════════════════════════════════
// I2C 드라이버
// ═══════════════════════════════════════════════════════

pub struct I2c {
    base: u32,
}

impl I2c {
    /// 새 I2C 인스턴스
    ///
    /// ```
    /// let i2c1 = unsafe { I2c::new(pac::addr::I2C1) };
    /// ```
    pub const unsafe fn new(base: u32) -> Self {
        Self { base }
    }

    // ── 레지스터 접근 헬퍼 ──

    #[inline(always)]
    fn read(&self, offset: u32) -> u32 {
        unsafe { pac::read_reg(self.base + offset) }
    }

    #[inline(always)]
    fn write(&self, offset: u32, val: u32) {
        unsafe { pac::write_reg(self.base + offset, val) }
    }

    // ═══════════════════════════════════════════════════
    // 초기화
    // ═══════════════════════════════════════════════════

    /// I2C 모듈 초기화 — Fast Mode (400kHz)
    ///
    /// 클럭 계산 (TRM 23.1.4.8):
    ///   FUNCCLK = 96 MHz (from L4_PER DPLL)
    ///   PSC = 7  → 내부 클럭 = 96MHz / (7+1) = 12 MHz
    ///   SCL = 400 kHz
    ///   SCLL = 12MHz / (2*400kHz) - 7 = 8
    ///   SCLH = 12MHz / (2*400kHz) - 5 = 10
    ///
    /// ```
    /// i2c.init_400khz();
    /// ```
    pub fn init_400khz(&self) {
        // 1. I2C 비활성화
        self.write(reg::CON, 0);

        // 2. 소프트 리셋
        self.write(reg::SYSC, sysc::SRST);

        // 리셋 완료 대기 (최대 ~10ms)
        let mut timeout = 100_000u32;
        while self.read(reg::SYSC) & sysc::SRST != 0 {
            timeout -= 1;
            if timeout == 0 {
                break;
            }
        }

        // 3. SYSC: Smart-idle + auto-idle + both clocks
        self.write(reg::SYSC, sysc::SMARTIDLE | sysc::AUTOIDLE | sysc::CLKACTIVITY_BOTH);

        // 4. 프리스케일러: 96MHz / (7+1) = 12MHz 내부 클럭
        self.write(reg::PSC, 7);

        // 5. SCL 타이밍: 400kHz Fast Mode
        //    SCLL: tLOW = (SCLL+7) * (1/12MHz)  → SCLL=8  → tLOW=1.25us
        //    SCLH: tHIGH = (SCLH+5) * (1/12MHz) → SCLH=10 → tHIGH=1.25us
        //    SCL period = 2.5us → 400kHz
        self.write(reg::SCLL, 8);
        self.write(reg::SCLH, 10);

        // 6. Own Address (Master — 아무 값이나 설정, 충돌 없도록)
        self.write(reg::OA, 0x01);

        // 7. FIFO 스레시홀드: TX=0, RX=0 (바이트 단위 인터럽트)
        self.write(reg::BUF, 0);

        // 8. 인터럽트 전체 클리어
        self.write(reg::IRQSTATUS, 0xFFFF);

        // 9. I2C 활성화 (Master 모드)
        self.write(reg::CON, con::I2C_EN);
    }

    /// I2C Standard Mode (100kHz) 초기화
    ///
    /// SCLL = 53, SCLH = 55 @ 12MHz 내부 클럭
    pub fn init_100khz(&self) {
        self.write(reg::CON, 0);
        self.write(reg::SYSC, sysc::SRST);

        let mut timeout = 100_000u32;
        while self.read(reg::SYSC) & sysc::SRST != 0 {
            timeout -= 1;
            if timeout == 0 { break; }
        }

        self.write(reg::SYSC, sysc::SMARTIDLE | sysc::AUTOIDLE | sysc::CLKACTIVITY_BOTH);
        self.write(reg::PSC, 7);

        // 100kHz: tLOW = (53+7)/12MHz = 5us, tHIGH = (55+5)/12MHz = 5us
        self.write(reg::SCLL, 53);
        self.write(reg::SCLH, 55);

        self.write(reg::OA, 0x01);
        self.write(reg::BUF, 0);
        self.write(reg::IRQSTATUS, 0xFFFF);
        self.write(reg::CON, con::I2C_EN);
    }

    // ═══════════════════════════════════════════════════
    // 버스 상태 확인
    // ═══════════════════════════════════════════════════

    /// 버스가 비어있을 때까지 대기 (Bus Busy 해제)
    fn wait_bus_free(&self) -> Result<(), I2cError> {
        let mut timeout = 50_000u32;
        while self.read(reg::IRQSTATUS_RAW) & irq::BB != 0 {
            timeout -= 1;
            if timeout == 0 {
                return Err(I2cError::BusBusy);
            }
        }
        Ok(())
    }

    /// 특정 IRQ 상태 비트 대기
    fn wait_for_flag(&self, flag: u32) -> Result<(), I2cError> {
        let mut timeout = 50_000u32;
        loop {
            let status = self.read(reg::IRQSTATUS_RAW);

            // 에러 체크
            if status & irq::NACK != 0 {
                self.write(reg::IRQSTATUS, irq::NACK);
                self.write(reg::CON, self.read(reg::CON) | con::STP);
                return Err(I2cError::Nack);
            }
            if status & irq::AL != 0 {
                self.write(reg::IRQSTATUS, irq::AL);
                return Err(I2cError::ArbitrationLost);
            }

            // 원하는 플래그 확인
            if status & flag != 0 {
                return Ok(());
            }

            timeout -= 1;
            if timeout == 0 {
                return Err(I2cError::Timeout);
            }
        }
    }

    // ═══════════════════════════════════════════════════
    // I2C 트랜잭션
    // ═══════════════════════════════════════════════════

    /// 슬레이브에 데이터 쓰기 (Master TX)
    ///
    /// ```
    /// // MPU6050 레지스터 0x6B에 0x00 쓰기 (Power Management)
    /// i2c.write_bytes(0x68, &[0x6B, 0x00])?;
    /// ```
    pub fn write_bytes(&self, slave_addr: u8, data: &[u8]) -> Result<(), I2cError> {
        if data.is_empty() {
            return Ok(());
        }

        // 1. 버스 대기
        self.wait_bus_free()?;

        // 2. 슬레이브 주소 설정
        self.write(reg::SA, slave_addr as u32);

        // 3. 데이터 카운터 설정
        self.write(reg::CNT, data.len() as u32);

        // 4. 이전 상태 클리어
        self.write(reg::IRQSTATUS, 0xFFFF);

        // 5. CON: Master + TX + Start + Stop
        self.write(reg::CON,
            con::I2C_EN | con::MST | con::TRX | con::STT | con::STP
        );

        // 6. 데이터 전송
        for &byte in data {
            self.wait_for_flag(irq::XRDY)?;
            self.write(reg::DATA, byte as u32);
            self.write(reg::IRQSTATUS, irq::XRDY);
        }

        // 7. 전송 완료 대기 (ARDY = Access Ready)
        self.wait_for_flag(irq::ARDY)?;
        self.write(reg::IRQSTATUS, irq::ARDY);

        Ok(())
    }

    /// 슬레이브에서 데이터 읽기 (Master RX)
    ///
    /// ```
    /// let mut buf = [0u8; 6];
    /// i2c.read_bytes(0x68, &mut buf)?; // 6바이트 읽기
    /// ```
    pub fn read_bytes(&self, slave_addr: u8, buf: &mut [u8]) -> Result<(), I2cError> {
        if buf.is_empty() {
            return Ok(());
        }

        self.wait_bus_free()?;
        self.write(reg::SA, slave_addr as u32);
        self.write(reg::CNT, buf.len() as u32);
        self.write(reg::IRQSTATUS, 0xFFFF);

        // CON: Master + RX + Start + Stop
        self.write(reg::CON,
            con::I2C_EN | con::MST | con::STT | con::STP
        );

        for byte in buf.iter_mut() {
            self.wait_for_flag(irq::RRDY)?;
            *byte = (self.read(reg::DATA) & 0xFF) as u8;
            self.write(reg::IRQSTATUS, irq::RRDY);
        }

        self.wait_for_flag(irq::ARDY)?;
        self.write(reg::IRQSTATUS, irq::ARDY);

        Ok(())
    }

    /// 레지스터 쓰기 (1바이트 레지스터 주소 + N바이트 데이터)
    ///
    /// ```
    /// // MPU6050: PWR_MGMT_1 (0x6B) = 0x00 (wake up)
    /// i2c.write_reg(0x68, 0x6B, &[0x00])?;
    /// ```
    pub fn write_reg(&self, slave_addr: u8, reg_addr: u8, data: &[u8]) -> Result<(), I2cError> {
        // 레지스터 주소 + 데이터를 하나의 트랜잭션으로
        let total_len = 1 + data.len();

        self.wait_bus_free()?;
        self.write(reg::SA, slave_addr as u32);
        self.write(reg::CNT, total_len as u32);
        self.write(reg::IRQSTATUS, 0xFFFF);
        self.write(reg::CON,
            con::I2C_EN | con::MST | con::TRX | con::STT | con::STP
        );

        // 레지스터 주소 전송
        self.wait_for_flag(irq::XRDY)?;
        self.write(reg::DATA, reg_addr as u32);
        self.write(reg::IRQSTATUS, irq::XRDY);

        // 데이터 전송
        for &byte in data {
            self.wait_for_flag(irq::XRDY)?;
            self.write(reg::DATA, byte as u32);
            self.write(reg::IRQSTATUS, irq::XRDY);
        }

        self.wait_for_flag(irq::ARDY)?;
        self.write(reg::IRQSTATUS, irq::ARDY);

        Ok(())
    }

    /// 레지스터 읽기 (Repeated Start 방식)
    ///
    /// I2C 프로토콜:
    ///   [S] [SlaveAddr+W] [RegAddr] [Sr] [SlaveAddr+R] [Data...] [P]
    ///
    /// ```
    /// let mut buf = [0u8; 14];
    /// // MPU6050: ACCEL_XOUT_H (0x3B)부터 14바이트 읽기
    /// i2c.read_reg(0x68, 0x3B, &mut buf)?;
    /// ```
    pub fn read_reg(&self, slave_addr: u8, reg_addr: u8, buf: &mut [u8]) -> Result<(), I2cError> {
        if buf.is_empty() {
            return Ok(());
        }

        // Phase 1: 레지스터 주소 쓰기 (Start, NO Stop)
        self.wait_bus_free()?;
        self.write(reg::SA, slave_addr as u32);
        self.write(reg::CNT, 1);
        self.write(reg::IRQSTATUS, 0xFFFF);

        // Master + TX + Start (Stop 없음 → Repeated Start 가능)
        self.write(reg::CON,
            con::I2C_EN | con::MST | con::TRX | con::STT
        );

        self.wait_for_flag(irq::XRDY)?;
        self.write(reg::DATA, reg_addr as u32);
        self.write(reg::IRQSTATUS, irq::XRDY);

        // ARDY 대기 (Phase 1 완료)
        self.wait_for_flag(irq::ARDY)?;
        self.write(reg::IRQSTATUS, irq::ARDY);

        // Phase 2: 데이터 읽기 (Repeated Start + Stop)
        self.write(reg::CNT, buf.len() as u32);
        self.write(reg::IRQSTATUS, 0xFFFF);

        // Master + RX + Start + Stop (Repeated Start)
        self.write(reg::CON,
            con::I2C_EN | con::MST | con::STT | con::STP
        );

        for byte in buf.iter_mut() {
            self.wait_for_flag(irq::RRDY)?;
            *byte = (self.read(reg::DATA) & 0xFF) as u8;
            self.write(reg::IRQSTATUS, irq::RRDY);
        }

        self.wait_for_flag(irq::ARDY)?;
        self.write(reg::IRQSTATUS, irq::ARDY);

        Ok(())
    }

    /// 모듈 리비전 읽기 (연결 테스트용)
    ///
    /// 정상이면 0x0050_0000 범위의 값이 반환됩니다
    pub fn read_revision(&self) -> u32 {
        let lo = self.read(reg::REVNB_LO) & 0xFFFF;
        let hi = self.read(reg::REVNB_HI) & 0xFFFF;
        (hi << 16) | lo
    }

    /// 슬레이브 존재 여부 확인 (ACK 테스트)
    ///
    /// ```
    /// if i2c.probe(0x68) {
    ///     // MPU6050 detected!
    /// }
    /// ```
    pub fn probe(&self, slave_addr: u8) -> bool {
        // 0바이트 쓰기 시도 — ACK만 확인
        self.wait_bus_free().ok();
        self.write(reg::SA, slave_addr as u32);
        self.write(reg::CNT, 0);
        self.write(reg::IRQSTATUS, 0xFFFF);
        self.write(reg::CON,
            con::I2C_EN | con::MST | con::TRX | con::STT | con::STP
        );

        // ARDY 또는 NACK 대기
        let mut timeout = 50_000u32;
        loop {
            let status = self.read(reg::IRQSTATUS_RAW);
            if status & irq::NACK != 0 {
                self.write(reg::IRQSTATUS, 0xFFFF);
                return false;
            }
            if status & irq::ARDY != 0 {
                self.write(reg::IRQSTATUS, 0xFFFF);
                return true;
            }
            timeout -= 1;
            if timeout == 0 {
                self.write(reg::IRQSTATUS, 0xFFFF);
                return false;
            }
        }
    }
}

// ═══════════════════════════════════════════════════════
// IMU 드라이버 (MPU6050 / ICM42688)
// ═══════════════════════════════════════════════════════

/// IMU 센서 타입
#[derive(Copy, Clone, Debug, PartialEq, Eq, defmt::Format)]
pub enum ImuType {
    /// InvenSense MPU-6050 (WHO_AM_I = 0x68)
    Mpu6050,
    /// TDK ICM-42688-P (WHO_AM_I = 0x47)
    Icm42688,
    /// Bosch BNO055 (CHIP_ID = 0xA0)
    Bno055,
    /// 알 수 없는 센서
    Unknown(u8),
}

/// IMU 6축 데이터 (가속도 + 자이로)
#[derive(Copy, Clone, Debug, Default, defmt::Format)]
pub struct ImuData {
    /// 가속도 X (raw, ±2g → 16384 LSB/g)
    pub accel_x: i16,
    /// 가속도 Y
    pub accel_y: i16,
    /// 가속도 Z
    pub accel_z: i16,
    /// 자이로 X (raw, ±250°/s → 131 LSB/°/s)
    pub gyro_x: i16,
    /// 자이로 Y
    pub gyro_y: i16,
    /// 자이로 Z
    pub gyro_z: i16,
    /// 온도 (raw)
    pub temperature: i16,
}

/// IMU 드라이버
pub struct Imu<'a> {
    i2c: &'a I2c,
    addr: u8,
    imu_type: ImuType,
}

/// MPU6050 레지스터 주소
pub mod mpu6050 {
    pub const WHO_AM_I: u8 = 0x75;
    pub const PWR_MGMT_1: u8 = 0x6B;
    pub const PWR_MGMT_2: u8 = 0x6C;
    pub const SMPLRT_DIV: u8 = 0x19;
    pub const CONFIG: u8 = 0x1A;
    pub const GYRO_CONFIG: u8 = 0x1B;
    pub const ACCEL_CONFIG: u8 = 0x1C;
    pub const INT_PIN_CFG: u8 = 0x37;
    pub const INT_ENABLE: u8 = 0x38;
    pub const ACCEL_XOUT_H: u8 = 0x3B;
    pub const TEMP_OUT_H: u8 = 0x41;
    pub const GYRO_XOUT_H: u8 = 0x43;

    pub const WHO_AM_I_VAL: u8 = 0x68;
}

/// ICM42688-P 레지스터 주소
pub mod icm42688 {
    pub const WHO_AM_I: u8 = 0x75;
    pub const PWR_MGMT0: u8 = 0x4E;
    pub const GYRO_CONFIG0: u8 = 0x4F;
    pub const ACCEL_CONFIG0: u8 = 0x50;
    pub const GYRO_CONFIG1: u8 = 0x51;
    pub const ACCEL_DATA_X1: u8 = 0x1F;
    pub const GYRO_DATA_X1: u8 = 0x25;
    pub const TEMP_DATA1: u8 = 0x1D;

    pub const WHO_AM_I_VAL: u8 = 0x47;
}

impl<'a> Imu<'a> {
    /// 새 IMU 인스턴스 — 자동 감지
    ///
    /// I2C 주소 0x68/0x69에서 WHO_AM_I를 읽어 센서 타입 판별
    pub fn new(i2c: &'a I2c) -> Option<Self> {
        // 0x68 먼저 시도
        for &addr in &[0x68u8, 0x69] {
            if i2c.probe(addr) {
                let mut who = [0u8];
                if i2c.read_reg(addr, mpu6050::WHO_AM_I, &mut who).is_ok() {
                    let imu_type = match who[0] {
                        mpu6050::WHO_AM_I_VAL => ImuType::Mpu6050,
                        icm42688::WHO_AM_I_VAL => ImuType::Icm42688,
                        0xA0 => ImuType::Bno055,
                        other => ImuType::Unknown(other),
                    };
                    return Some(Self { i2c, addr, imu_type });
                }
            }
        }
        None
    }

    /// 센서 타입 반환
    pub fn sensor_type(&self) -> ImuType {
        self.imu_type
    }

    /// I2C 주소 반환
    pub fn address(&self) -> u8 {
        self.addr
    }

    /// IMU 초기화 (센서별 설정)
    pub fn init(&self) -> Result<(), I2cError> {
        match self.imu_type {
            ImuType::Mpu6050 => self.init_mpu6050(),
            ImuType::Icm42688 => self.init_icm42688(),
            _ => Ok(()), // Unknown — 기본 설정 유지
        }
    }

    /// MPU6050 초기화
    ///
    /// - Clock source: PLL with X Gyro
    /// - Gyro: ±250°/s (131 LSB/°/s)
    /// - Accel: ±2g (16384 LSB/g)
    /// - Sample Rate: 1kHz (DLPF 44Hz)
    fn init_mpu6050(&self) -> Result<(), I2cError> {
        // 1. 리셋
        self.i2c.write_reg(self.addr, mpu6050::PWR_MGMT_1, &[0x80])?;
        // 리셋 완료 대기 (~100ms)
        for _ in 0..100_000 { cortex_m::asm::nop(); }

        // 2. Wake up + X Gyro PLL
        self.i2c.write_reg(self.addr, mpu6050::PWR_MGMT_1, &[0x01])?;

        // 3. Sample Rate Divider: 1kHz / (0+1) = 1kHz
        self.i2c.write_reg(self.addr, mpu6050::SMPLRT_DIV, &[0])?;

        // 4. DLPF: 44Hz bandwidth (CONFIG = 3)
        self.i2c.write_reg(self.addr, mpu6050::CONFIG, &[3])?;

        // 5. Gyro: ±250°/s (FS_SEL = 0)
        self.i2c.write_reg(self.addr, mpu6050::GYRO_CONFIG, &[0x00])?;

        // 6. Accel: ±2g (AFS_SEL = 0)
        self.i2c.write_reg(self.addr, mpu6050::ACCEL_CONFIG, &[0x00])?;

        // 7. Data Ready 인터럽트 활성화
        self.i2c.write_reg(self.addr, mpu6050::INT_ENABLE, &[0x01])?;

        Ok(())
    }

    /// ICM42688-P 초기화
    ///
    /// - Gyro: ±250°/s, 1kHz ODR
    /// - Accel: ±2g, 1kHz ODR
    /// - Low Noise Mode
    fn init_icm42688(&self) -> Result<(), I2cError> {
        // 1. Accel + Gyro → Low Noise Mode
        self.i2c.write_reg(self.addr, icm42688::PWR_MGMT0, &[0x0F])?;
        for _ in 0..50_000 { cortex_m::asm::nop(); }

        // 2. Gyro Config: ±250°/s, 1kHz ODR
        self.i2c.write_reg(self.addr, icm42688::GYRO_CONFIG0, &[0x06])?;

        // 3. Accel Config: ±2g, 1kHz ODR
        self.i2c.write_reg(self.addr, icm42688::ACCEL_CONFIG0, &[0x06])?;

        Ok(())
    }

    /// 6축 데이터 읽기 (Accel + Temp + Gyro)
    ///
    /// MPU6050: Burst read from 0x3B (14 bytes)
    ///   [AccelX_H, AccelX_L, AccelY_H, AccelY_L, AccelZ_H, AccelZ_L,
    ///    Temp_H, Temp_L,
    ///    GyroX_H, GyroX_L, GyroY_H, GyroY_L, GyroZ_H, GyroZ_L]
    pub fn read_all(&self) -> Result<ImuData, I2cError> {
        match self.imu_type {
            ImuType::Mpu6050 => self.read_mpu6050(),
            ImuType::Icm42688 => self.read_icm42688(),
            _ => Ok(ImuData::default()),
        }
    }

    fn read_mpu6050(&self) -> Result<ImuData, I2cError> {
        let mut buf = [0u8; 14];
        self.i2c.read_reg(self.addr, mpu6050::ACCEL_XOUT_H, &mut buf)?;

        Ok(ImuData {
            accel_x: i16::from_be_bytes([buf[0], buf[1]]),
            accel_y: i16::from_be_bytes([buf[2], buf[3]]),
            accel_z: i16::from_be_bytes([buf[4], buf[5]]),
            temperature: i16::from_be_bytes([buf[6], buf[7]]),
            gyro_x: i16::from_be_bytes([buf[8], buf[9]]),
            gyro_y: i16::from_be_bytes([buf[10], buf[11]]),
            gyro_z: i16::from_be_bytes([buf[12], buf[13]]),
        })
    }

    fn read_icm42688(&self) -> Result<ImuData, I2cError> {
        // ICM42688: Temp(2) + Accel(6) + Gyro(6) = 14 bytes from 0x1D
        let mut buf = [0u8; 14];
        self.i2c.read_reg(self.addr, icm42688::TEMP_DATA1, &mut buf)?;

        Ok(ImuData {
            temperature: i16::from_be_bytes([buf[0], buf[1]]),
            accel_x: i16::from_be_bytes([buf[2], buf[3]]),
            accel_y: i16::from_be_bytes([buf[4], buf[5]]),
            accel_z: i16::from_be_bytes([buf[6], buf[7]]),
            gyro_x: i16::from_be_bytes([buf[8], buf[9]]),
            gyro_y: i16::from_be_bytes([buf[10], buf[11]]),
            gyro_z: i16::from_be_bytes([buf[12], buf[13]]),
        })
    }

    /// 가속도만 읽기 (6 bytes)
    pub fn read_accel(&self) -> Result<[i16; 3], I2cError> {
        let data = self.read_all()?;
        Ok([data.accel_x, data.accel_y, data.accel_z])
    }

    /// 자이로만 읽기 (6 bytes)
    pub fn read_gyro(&self) -> Result<[i16; 3], I2cError> {
        let data = self.read_all()?;
        Ok([data.gyro_x, data.gyro_y, data.gyro_z])
    }
}
