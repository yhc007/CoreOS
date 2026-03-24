//! ─────────────────────────────────────────────────────────────
//! AM5729 ePWM (Enhanced Pulse Width Modulation) 드라이버
//! ─────────────────────────────────────────────────────────────
//!
//! TRM SPRUHZ6 — Chapter 23: ePWM
//! 로봇 모터 제어의 핵심: DC 모터, BLDC, 서보 드라이버
//!
//! ePWM 모듈 구조:
//!   Time-Base (TB)  → 카운터 / 주기 설정
//!   Counter-Compare (CC) → 듀티 사이클 (CMPA, CMPB)
//!   Action-Qualifier (AQ) → 출력 액션 (set/clear/toggle)
//!   Dead-Band (DB)  → H-브리지 shoot-through 방지
//!   Trip-Zone (TZ)  → 과전류/과열 비상 정지

/// ePWM 레지스터 오프셋 (모듈 베이스 기준)
pub mod offset {
    // ── Time-Base ──
    pub const TBCTL: u32 = 0x00;   // TB Control
    pub const TBSTS: u32 = 0x02;   // TB Status
    pub const TBPHS: u32 = 0x06;   // TB Phase
    pub const TBCNT: u32 = 0x08;   // TB Counter
    pub const TBPRD: u32 = 0x0A;   // TB Period

    // ── Counter-Compare ──
    pub const CMPCTL: u32 = 0x0E;  // CC Control
    pub const CMPA: u32 = 0x12;    // Compare A
    pub const CMPB: u32 = 0x14;    // Compare B

    // ── Action-Qualifier ──
    pub const AQCTLA: u32 = 0x16;  // AQ Output A Control
    pub const AQCTLB: u32 = 0x18;  // AQ Output B Control

    // ── Dead-Band ──
    pub const DBCTL: u32 = 0x1E;   // DB Control
    pub const DBRED: u32 = 0x20;   // DB Rising Edge Delay
    pub const DBFED: u32 = 0x22;   // DB Falling Edge Delay

    // ── Trip-Zone ──
    pub const TZSEL: u32 = 0x24;   // TZ Select
    pub const TZCTL: u32 = 0x28;   // TZ Control
    pub const TZEINT: u32 = 0x2A;  // TZ Enable Interrupt
    pub const TZFLG: u32 = 0x2C;   // TZ Flag
    pub const TZCLR: u32 = 0x2E;   // TZ Clear

    // ── Event-Trigger ──
    pub const ETSEL: u32 = 0x32;   // ET Event Selection
    pub const ETPS: u32 = 0x34;    // ET Pre-Scale
    pub const ETFLG: u32 = 0x36;   // ET Flag
    pub const ETCLR: u32 = 0x38;   // ET Clear
}

/// TBCTL 비트 필드
pub mod tbctl {
    /// 카운트 방향
    pub const CTRMODE_UP: u16 = 0;        // Up-count
    pub const CTRMODE_DOWN: u16 = 1;      // Down-count
    pub const CTRMODE_UPDOWN: u16 = 2;    // Up-down-count (center-aligned)

    /// 클럭 분주비 (CLKDIV)
    pub const CLKDIV_1: u16 = 0 << 10;
    pub const CLKDIV_2: u16 = 1 << 10;
    pub const CLKDIV_4: u16 = 2 << 10;
    pub const CLKDIV_8: u16 = 3 << 10;

    /// High-speed 클럭 분주비 (HSPCLKDIV)
    pub const HSPCLKDIV_1: u16 = 0 << 7;
    pub const HSPCLKDIV_2: u16 = 1 << 7;
    pub const HSPCLKDIV_4: u16 = 2 << 7;

    /// Phase Direction
    pub const PHSDIR_DOWN: u16 = 0 << 13;
    pub const PHSDIR_UP: u16 = 1 << 13;
}

/// AQCTLA/B 액션 값
pub mod aqctl {
    /// CMPA 매치 시 Up-count 액션 (ZRO, PRD, CAU, CAD, CBU, CBD)
    pub const CAU_SET: u16 = 2 << 4;     // CMPA Up → Set High
    pub const CAU_CLEAR: u16 = 1 << 4;   // CMPA Up → Set Low
    pub const CAU_TOGGLE: u16 = 3 << 4;  // CMPA Up → Toggle

    pub const ZRO_SET: u16 = 2 << 0;     // Zero → Set High
    pub const ZRO_CLEAR: u16 = 1 << 0;   // Zero → Set Low

    pub const PRD_SET: u16 = 2 << 2;     // Period → Set High
    pub const PRD_CLEAR: u16 = 1 << 2;   // Period → Set Low

    pub const CAD_SET: u16 = 2 << 6;     // CMPA Down → Set High
    pub const CAD_CLEAR: u16 = 1 << 6;   // CMPA Down → Set Low

    pub const CBU_SET: u16 = 2 << 8;     // CMPB Up → Set High
    pub const CBU_CLEAR: u16 = 1 << 8;   // CMPB Up → Set Low
}

/// Dead-Band 모드
pub mod dbctl {
    pub const OUT_MODE_BYPASS: u16 = 0;   // Dead-band 비활성화
    pub const OUT_MODE_A_B: u16 = 3;      // Active High Complementary
    pub const POLSEL_AH: u16 = 2 << 2;    // Active High
    pub const POLSEL_ALC: u16 = 1 << 2;   // Active Low Complementary
    pub const IN_MODE_A: u16 = 0 << 4;    // A 신호로 양쪽 생성
}

/// ePWM 드라이버
pub struct Epwm {
    base: u32,
}

impl Epwm {
    pub const unsafe fn new(base: u32) -> Self {
        Self { base }
    }

    /// 16-bit 레지스터 읽기
    unsafe fn read16(&self, offset: u32) -> u16 {
        core::ptr::read_volatile((self.base + offset) as *const u16)
    }

    /// 16-bit 레지스터 쓰기
    unsafe fn write16(&self, offset: u32, val: u16) {
        core::ptr::write_volatile((self.base + offset) as *mut u16, val);
    }

    /// 모터 제어용 PWM 초기화
    ///
    /// - `freq_hz`: PWM 주파수 (예: 20_000 = 20kHz for DC motor)
    /// - `sysclk_hz`: 시스템 클럭 (기본 20MHz for PWMSS)
    /// - Up-down count (center-aligned) + Dead-band + Trip-zone
    pub fn init_motor_pwm(&self, freq_hz: u32, sysclk_hz: u32) {
        unsafe {
            // 1. Time-Base 설정
            // Up-Down count → freq = sysclk / (2 * TBPRD)
            // TBPRD = sysclk / (2 * freq)
            let tbprd = (sysclk_hz / (2 * freq_hz)) as u16;

            let tbctl = tbctl::CTRMODE_UPDOWN
                      | tbctl::CLKDIV_1
                      | tbctl::HSPCLKDIV_1;
            self.write16(offset::TBCTL, tbctl);
            self.write16(offset::TBPRD, tbprd);
            self.write16(offset::TBCNT, 0);
            self.write16(offset::TBPHS, 0);

            // 2. Action-Qualifier (Center-Aligned PWM)
            // A 출력: CMPA Up → Clear, CMPA Down → Set
            let aqctla = aqctl::CAU_CLEAR | aqctl::CAD_SET;
            self.write16(offset::AQCTLA, aqctla);

            // B 출력: CMPB Up → Clear, CMPB Down → Set
            let aqctlb = aqctl::CBU_CLEAR;
            self.write16(offset::AQCTLB, aqctlb);

            // 3. Dead-Band (H-브리지 shoot-through 방지)
            // 1µs dead-time @ 20MHz = 20 counts
            let dead_counts = (sysclk_hz / 1_000_000) as u16; // 1µs
            let dbctl_val = dbctl::OUT_MODE_A_B | dbctl::POLSEL_AH | dbctl::IN_MODE_A;
            self.write16(offset::DBCTL, dbctl_val);
            self.write16(offset::DBRED, dead_counts);
            self.write16(offset::DBFED, dead_counts);

            // 4. 초기 듀티 = 0 (정지)
            self.write16(offset::CMPA, 0);
            self.write16(offset::CMPB, 0);
        }
    }

    /// 서보 모터용 PWM 초기화 (50Hz, 1~2ms 펄스)
    pub fn init_servo_pwm(&self, sysclk_hz: u32) {
        unsafe {
            // 50Hz → TBPRD = sysclk / (50 * 2 * prescaler)
            // prescaler = 8 → TBPRD = 20MHz / (50 * 2 * 8) = 25000
            let tbctl = tbctl::CTRMODE_UPDOWN
                      | tbctl::CLKDIV_8
                      | tbctl::HSPCLKDIV_1;
            let tbprd = (sysclk_hz / (50 * 2 * 8)) as u16;

            self.write16(offset::TBCTL, tbctl);
            self.write16(offset::TBPRD, tbprd);
            self.write16(offset::TBCNT, 0);

            // Up-count only PWM for servo
            let aqctla = aqctl::ZRO_SET | aqctl::CAU_CLEAR;
            self.write16(offset::AQCTLA, aqctla);

            // Dead-band 불필요 (서보)
            self.write16(offset::DBCTL, dbctl::OUT_MODE_BYPASS);

            // 중립 위치 (1.5ms)
            self.set_servo_angle(90);
        }
    }

    /// 듀티 사이클 설정 (0~1000 = 0.0%~100.0%)
    pub fn set_duty_permille(&self, duty: u16) {
        unsafe {
            let tbprd = self.read16(offset::TBPRD);
            let cmpa = ((tbprd as u32) * (duty as u32) / 1000) as u16;
            self.write16(offset::CMPA, cmpa);
        }
    }

    /// 서보 각도 설정 (0~180도)
    /// 펄스폭: 1ms (0°) ~ 2ms (180°) → 1.5ms = 90° (중립)
    pub fn set_servo_angle(&self, angle_deg: u16) {
        let angle = angle_deg.min(180);
        unsafe {
            let tbprd = self.read16(offset::TBPRD);
            // 1ms ~ 2ms → TBPRD의 5% ~ 10% (@ 50Hz)
            // duty = TBPRD * (0.05 + angle/180 * 0.05)
            let min_duty = (tbprd as u32) * 5 / 100;  // 1ms = 5%
            let range = (tbprd as u32) * 5 / 100;     // 1ms range
            let cmpa = min_duty + range * (angle as u32) / 180;
            self.write16(offset::CMPA, cmpa as u16);
        }
    }

    /// Trip-Zone 비상 정지 (모든 출력 LOW)
    pub fn emergency_stop(&self) {
        unsafe {
            // One-Shot Trip → 양쪽 출력 Force Low
            let tzctl = 2 | (2 << 2); // TZA=Force Low, TZB=Force Low
            self.write16(offset::TZCTL, tzctl);
            // 소프트웨어 Trip 트리거
            self.write16(offset::TZCLR, 0x4); // OST 클리어 비트
        }
    }

    /// 카운터 현재 값 읽기
    pub fn read_counter(&self) -> u16 {
        unsafe { self.read16(offset::TBCNT) }
    }
}
