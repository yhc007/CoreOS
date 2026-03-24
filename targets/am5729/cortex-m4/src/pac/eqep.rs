//! ─────────────────────────────────────────────────────────────
//! AM5729 eQEP (Enhanced Quadrature Encoder Pulse) 드라이버
//! ─────────────────────────────────────────────────────────────
//!
//! TRM SPRUHZ6 — Chapter 23: eQEP
//! 로봇 관절/모터의 위치 및 속도 피드백
//!
//! 인코더 신호:
//!   Phase A ─┐
//!   Phase B ─┼─→ eQEP → Position Counter (32-bit)
//!   Index   ─┘         → Speed (period/frequency)
//!
//! 기능:
//!   - 32-bit 위치 카운터 (최대 4,294,967,295 카운트)
//!   - 방향 감지 (A/B 위상 관계)
//!   - 속도 측정 (period capture / frequency measurement)
//!   - Index 펄스로 절대 위치 리셋
//!   - Watchdog (인코더 신호 끊김 감지)

/// eQEP 레지스터 오프셋
pub mod offset {
    pub const QPOSCNT: u32 = 0x00;   // Position Counter
    pub const QPOSINIT: u32 = 0x04;  // Position Init
    pub const QPOSMAX: u32 = 0x08;   // Maximum Position Count
    pub const QPOSCMP: u32 = 0x0C;   // Position Compare
    pub const QPOSILAT: u32 = 0x10;  // Index Position Latch
    pub const QPOSSLAT: u32 = 0x14;  // Strobe Position Latch
    pub const QPOSLAT: u32 = 0x18;   // Position Counter Latch

    pub const QUTMR: u32 = 0x1C;     // Unit Timer
    pub const QUPRD: u32 = 0x20;     // Unit Period

    pub const QWDTMR: u32 = 0x24;    // Watchdog Timer
    pub const QWDPRD: u32 = 0x26;    // Watchdog Period

    pub const QDECCTL: u32 = 0x28;   // Decoder Control
    pub const QEPCTL: u32 = 0x2A;    // QEP Control
    pub const QCAPCTL: u32 = 0x2C;   // Capture Control
    pub const QPOSCTL: u32 = 0x2E;   // Position-Compare Control

    pub const QEINT: u32 = 0x30;     // Interrupt Enable
    pub const QFLG: u32 = 0x32;      // Interrupt Flag
    pub const QCLR: u32 = 0x34;      // Interrupt Clear
    pub const QFRC: u32 = 0x36;      // Interrupt Force

    pub const QEPSTS: u32 = 0x38;    // QEP Status

    pub const QCTMR: u32 = 0x3A;     // Capture Timer
    pub const QCPRD: u32 = 0x3C;     // Capture Period
    pub const QCTMRLAT: u32 = 0x3E;  // Capture Timer Latch
    pub const QCPRDLAT: u32 = 0x40;  // Capture Period Latch
}

/// QDECCTL 비트
pub mod qdecctl {
    pub const QSRC_QUAD: u16 = 0 << 14;    // Quadrature count mode
    pub const QSRC_DIR: u16 = 1 << 14;     // Direction count mode
    pub const QSRC_UP: u16 = 2 << 14;      // Up count mode
    pub const QSRC_DOWN: u16 = 3 << 14;    // Down count mode
    pub const SOEN: u16 = 1 << 13;         // Sync output enable
    pub const SWAP: u16 = 1 << 12;         // Swap A/B inputs
    pub const IGATE: u16 = 1 << 11;        // Index pulse gating
    pub const QAP: u16 = 1 << 9;           // A input polarity (1=invert)
    pub const QBP: u16 = 1 << 8;           // B input polarity (1=invert)
    pub const QIP: u16 = 1 << 7;           // Index polarity (1=invert)
    pub const QSP: u16 = 1 << 6;           // Strobe polarity (1=invert)
}

/// QEPCTL 비트
pub mod qepctl {
    pub const QPEN: u16 = 1 << 15;         // QEP 활성화
    pub const SWI: u16 = 1 << 14;          // SW init position counter
    pub const SEL: u16 = 1 << 13;          // Strobe event latch
    pub const IEL_RISING: u16 = 1 << 12;   // Index event latch (rising)
    pub const IEL_FALLING: u16 = 2 << 12;  // Index event latch (falling)
    pub const PCRM_INDEX: u16 = 0 << 8;    // Position counter reset on index
    pub const PCRM_MAX: u16 = 1 << 8;      // Reset on max position
    pub const PCRM_FIRST_INDEX: u16 = 2 << 8; // Reset on first index
    pub const PCRM_UNIT_TIME: u16 = 3 << 8; // Reset on unit time event
    pub const UTE: u16 = 1 << 1;           // Unit timer enable
    pub const WDE: u16 = 1 << 0;           // Watchdog enable
}

/// QCAPCTL 비트
pub mod qcapctl {
    pub const CEN: u16 = 1 << 15;          // Capture enable
    pub const CCPS_1: u16 = 0 << 4;        // Capture clock prescaler /1
    pub const CCPS_2: u16 = 1 << 4;        // /2
    pub const CCPS_4: u16 = 2 << 4;        // /4
    pub const CCPS_8: u16 = 3 << 4;        // /8
    pub const CCPS_16: u16 = 4 << 4;       // /16
    pub const CCPS_32: u16 = 5 << 4;       // /32
    pub const CCPS_64: u16 = 6 << 4;       // /64
    pub const CCPS_128: u16 = 7 << 4;      // /128
    pub const UPPS_1: u16 = 0;             // Unit position prescaler /1
    pub const UPPS_4: u16 = 2;             // /4
    pub const UPPS_16: u16 = 4;            // /16
}

/// eQEP 드라이버
pub struct Eqep {
    base: u32,
}

impl Eqep {
    pub const unsafe fn new(base: u32) -> Self {
        Self { base }
    }

    unsafe fn read16(&self, offset: u32) -> u16 {
        core::ptr::read_volatile((self.base + offset) as *const u16)
    }

    unsafe fn write16(&self, offset: u32, val: u16) {
        core::ptr::write_volatile((self.base + offset) as *mut u16, val);
    }

    unsafe fn read32(&self, offset: u32) -> u32 {
        core::ptr::read_volatile((self.base + offset) as *const u32)
    }

    unsafe fn write32(&self, offset: u32, val: u32) {
        core::ptr::write_volatile((self.base + offset) as *mut u32, val);
    }

    /// 인코더 초기화
    ///
    /// - `counts_per_rev`: 인코더 1회전당 카운트 수 (4x quadrature)
    /// - `sysclk_hz`: 시스템 클럭 (20MHz)
    pub fn init(&self, counts_per_rev: u32, sysclk_hz: u32) {
        unsafe {
            // 1. Decoder 설정: Quadrature count mode
            self.write16(offset::QDECCTL, qdecctl::QSRC_QUAD);

            // 2. QEP Control: 활성화 + Index reset + Unit timer
            let qepctl = qepctl::QPEN
                        | qepctl::IEL_RISING
                        | qepctl::PCRM_INDEX   // Index 펄스에서 카운터 리셋
                        | qepctl::UTE           // Unit timer 활성화
                        | qepctl::WDE;          // Watchdog 활성화
            self.write16(offset::QEPCTL, qepctl);

            // 3. Maximum position = counts per revolution
            self.write32(offset::QPOSMAX, counts_per_rev);

            // 4. Unit timer period (100ms → 속도 측정 주기)
            // QUPRD = sysclk * 0.1 = 20MHz * 0.1 = 2,000,000
            self.write32(offset::QUPRD, sysclk_hz / 10);

            // 5. Capture 설정 (고속/저속 겸용)
            let qcapctl = qcapctl::CEN | qcapctl::CCPS_8 | qcapctl::UPPS_4;
            self.write16(offset::QCAPCTL, qcapctl);

            // 6. Watchdog period (100ms timeout → 인코더 끊김 감지)
            self.write16(offset::QWDPRD, (sysclk_hz / 10 / 64) as u16);

            // 7. 초기 위치 = 0
            self.write32(offset::QPOSCNT, 0);
            self.write32(offset::QPOSINIT, 0);

            // 8. 인터럽트 활성화 (Unit time event + Watchdog timeout)
            self.write16(offset::QEINT, (1 << 11) | (1 << 8)); // UTO + WTO
        }
    }

    /// 현재 위치 읽기 (raw counts)
    pub fn read_position(&self) -> u32 {
        unsafe { self.read32(offset::QPOSCNT) }
    }

    /// 현재 위치 읽기 (도 단위, 0.0~360.0)
    pub fn read_position_deg(&self, counts_per_rev: u32) -> f32 {
        let pos = self.read_position();
        (pos as f32 / counts_per_rev as f32) * 360.0
    }

    /// 속도 읽기 (RPM)
    ///
    /// Unit Time Event에서 래치된 위치 차이로 계산
    /// RPM = (Δcount / counts_per_rev) * (60 / unit_time_sec)
    pub fn read_speed_rpm(&self, counts_per_rev: u32) -> f32 {
        unsafe {
            let pos_latch = self.read32(offset::QPOSLAT);
            let pos_current = self.read32(offset::QPOSCNT);

            let delta = if pos_current >= pos_latch {
                pos_current - pos_latch
            } else {
                // 카운터 롤오버
                counts_per_rev - pos_latch + pos_current
            };

            // Unit time = 100ms → 1초당 10회
            let revolutions = delta as f32 / counts_per_rev as f32;
            revolutions * 10.0 * 60.0 // RPM
        }
    }

    /// 방향 읽기 (true = 정방향, false = 역방향)
    pub fn read_direction(&self) -> bool {
        unsafe {
            let status = self.read16(offset::QEPSTS);
            (status & 0x02) != 0 // QDF bit
        }
    }

    /// 위치 카운터 리셋
    pub fn reset_position(&self) {
        unsafe {
            self.write32(offset::QPOSCNT, 0);
        }
    }

    /// Watchdog 타임아웃 확인 (인코더 신호 끊김)
    pub fn is_watchdog_timeout(&self) -> bool {
        unsafe {
            let flags = self.read16(offset::QFLG);
            (flags & (1 << 8)) != 0 // WTO bit
        }
    }

    /// 인터럽트 플래그 클리어
    pub fn clear_flags(&self) {
        unsafe {
            self.write16(offset::QCLR, 0xFFFF);
        }
    }
}
