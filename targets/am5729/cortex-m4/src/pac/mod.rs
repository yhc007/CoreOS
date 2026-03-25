//! ─────────────────────────────────────────────────────────────
//! AM5729 IPU1 Cortex-M4 — Minimal PAC (Peripheral Access Crate)
//! ─────────────────────────────────────────────────────────────
//!
//! TI AM572x TRM (SPRUHZ6) 기반 수동 레지스터 매핑.
//! SVD 파일이 공개되지 않아 핵심 페리페럴만 직접 정의합니다.
//!
//! 주의: IPU는 AMMU를 통해 주소 변환을 수행합니다.
//!   L4_PER 영역(0x4800_0000)은 AMMU large page로 직접 매핑해야 합니다.
//!
//! 참고: TRM SPRUHZ6 Chapter 2 (Memory Map), Chapter 24-26 (Peripherals)

pub mod ammu;
pub mod intc;
pub mod i2c;
pub mod gpio;
pub mod timer;
pub mod uart;
pub mod dcan;
pub mod epwm;
pub mod eqep;
pub mod cpsw;
pub mod pru;

// ── L4 페리페럴 베이스 주소 (물리 주소) ──
// IPU AMMU가 1:1 매핑하도록 설정한 후 이 주소 사용

/// GPIO 모듈 베이스 주소 (TRM Table 2-3)
pub mod addr {
    pub const GPIO1: u32 = 0x4AE1_0000; // L4_WKUP
    pub const GPIO2: u32 = 0x4805_5000; // L4_PER1
    pub const GPIO3: u32 = 0x4805_7000;
    pub const GPIO4: u32 = 0x4805_9000;
    pub const GPIO5: u32 = 0x4805_B000;
    pub const GPIO6: u32 = 0x4805_D000;
    pub const GPIO7: u32 = 0x4805_1000;
    pub const GPIO8: u32 = 0x4805_3000;

    /// UART 모듈 베이스 주소
    pub const UART1: u32 = 0x4806_A000;
    pub const UART2: u32 = 0x4806_C000;
    pub const UART3: u32 = 0x4802_0000;

    /// GP Timer 베이스 주소 (DMTIMER)
    pub const TIMER2: u32 = 0x4803_2000;
    pub const TIMER3: u32 = 0x4803_4000;
    pub const TIMER4: u32 = 0x4803_6000;

    /// DCAN (Controller Area Network) 베이스 주소
    pub const DCAN1: u32 = 0x481C_C000;
    pub const DCAN2: u32 = 0x481D_0000;

    /// I2C 베이스 주소
    pub const I2C1: u32 = 0x4807_0000;
    pub const I2C2: u32 = 0x4807_2000;
    pub const I2C3: u32 = 0x4806_0000;

    /// ePWM 베이스 주소
    pub const EPWM1: u32 = 0x4843_E200;
    pub const EPWM2: u32 = 0x4844_0200;
    pub const EPWM3: u32 = 0x4844_2200;

    /// eQEP (Quadrature Encoder) 베이스 주소
    pub const EQEP1: u32 = 0x4843_E180;
    pub const EQEP2: u32 = 0x4844_0180;
    pub const EQEP3: u32 = 0x4844_2180;

    /// Mailbox (A15 ↔ M4 IPC) 베이스 주소
    pub const MAILBOX5: u32 = 0x4883_A000; // IPU1 ↔ MPU
    pub const MAILBOX6: u32 = 0x4883_C000; // IPU1 ↔ DSP1

    /// CPSW (Common Platform Switch) — Ethernet TSN
    pub const CPSW: u32 = 0x4848_0000;

    /// PRU-ICSS (Programmable Real-time Unit) — Industrial Ethernet
    pub const PRU_ICSS1: u32 = 0x4B20_0000;
    pub const PRU_ICSS2: u32 = 0x4B28_0000;
}

/// 레지스터 읽기 헬퍼 (volatile)
#[inline(always)]
pub unsafe fn read_reg(addr: u32) -> u32 {
    core::ptr::read_volatile(addr as *const u32)
}

/// 레지스터 쓰기 헬퍼 (volatile)
#[inline(always)]
pub unsafe fn write_reg(addr: u32, val: u32) {
    core::ptr::write_volatile(addr as *mut u32, val);
}

/// 레지스터 비트 세트 헬퍼
#[inline(always)]
pub unsafe fn set_bits(addr: u32, mask: u32) {
    let val = read_reg(addr);
    write_reg(addr, val | mask);
}

/// 레지스터 비트 클리어 헬퍼
#[inline(always)]
pub unsafe fn clear_bits(addr: u32, mask: u32) {
    let val = read_reg(addr);
    write_reg(addr, val & !mask);
}
