//! ─────────────────────────────────────────────────────────────
//! AM5729 IPU1 Cortex-M4 — Interrupt Controller & RTIC Dispatchers
//! ─────────────────────────────────────────────────────────────
//!
//! AM5729 IPU1은 Cortex-M4F 듀얼 코어로, 표준 ARM NVIC를 사용합니다.
//! IPU 내부 INTC (Crossbar를 통해)가 L4 인터럽트를 NVIC IRQ에 매핑합니다.
//!
//! RTIC v2는 dispatchers에 "사용하지 않는 인터럽트 번호"가 필요합니다.
//! AM5729 IPU1은 최대 64개의 IRQ를 지원하며, INTC Crossbar를 통해
//! 400+ 시스템 인터럽트 중 일부를 NVIC IRQ에 할당합니다.
//!
//! ═══════════════════════════════════════════════════════════════
//!  RTIC Dispatcher 전략
//! ═══════════════════════════════════════════════════════════════
//!
//! RTIC v2 `dispatchers = [IRQ_NAME, ...]`는 사용하지 않는 NVIC IRQ를
//! 소프트웨어 태스크 디스패치용으로 "빌려"씁니다.
//!
//! AM5729 IPU1에서 사용 가능한 "예비" IRQ:
//!   - IRQ 55~63: 대부분 미할당 (Crossbar default = Reserved)
//!   - 우리 펌웨어가 사용하는 IRQ:
//!     * IRQ 26: Mailbox5 User2 (rpmsg 수신)
//!     * IRQ 23: TIMER2 overflow
//!     * IRQ 33: ePWM trip-zone
//!   - RTIC dispatchers로 사용할 예비 IRQ:
//!     * IRQ 55: DISPATCH_SW0 (priority 1 tasks)
//!     * IRQ 56: DISPATCH_SW1 (priority 2 tasks)
//!     * IRQ 57: DISPATCH_SW2 (priority 3 tasks)
//!     * IRQ 58: DISPATCH_SW3 (priority 4 tasks — motor control)
//!
//! ═══════════════════════════════════════════════════════════════
//!  IPU1 NVIC IRQ 할당 테이블
//! ═══════════════════════════════════════════════════════════════
//!
//! | IRQ# | 인터럽트 소스           | 용도                |
//! |------|------------------------|---------------------|
//! |  23  | TIMER2_IRQ             | GP Timer (미사용)   |
//! |  26  | MAILBOX5_USER2_IRQ     | rpmsg 수신          |
//! |  33  | EPWM_TZ_IRQ            | 모터 trip-zone      |
//! |  55  | (Reserved/Crossbar)    | RTIC Dispatch SW0   |
//! |  56  | (Reserved/Crossbar)    | RTIC Dispatch SW1   |
//! |  57  | (Reserved/Crossbar)    | RTIC Dispatch SW2   |
//! |  58  | (Reserved/Crossbar)    | RTIC Dispatch SW3   |
//!
//! 참고: TRM SPRUHZ6 — Chapter 6: Interrupt Controller,
//!       Table 6-53: IPU1 Default Interrupt Mapping

use crate::pac;

// ═══════════════════════════════════════════════════════
// IPU1 NVIC IRQ 번호 정의
// ═══════════════════════════════════════════════════════

/// RTIC dispatcher용 예비 IRQ 번호 (Crossbar 미할당)
///
/// AM5729 IPU INTC는 Crossbar를 통해 시스템 인터럽트를 NVIC IRQ에
/// 동적으로 매핑합니다. 이 IRQ 번호들은 Crossbar에서 아무 소스도
/// 연결하지 않으므로 하드웨어에서 트리거되지 않습니다.
/// RTIC가 NVIC_ISPR(Software Trigger)로 소프트웨어 pend 합니다.
pub const DISPATCH_SW0: u16 = 55; // RTIC priority 1
pub const DISPATCH_SW1: u16 = 56; // RTIC priority 2
pub const DISPATCH_SW2: u16 = 57; // RTIC priority 3
pub const DISPATCH_SW3: u16 = 58; // RTIC priority 4

/// Mailbox5 User2 IRQ (A15 → M4 rpmsg 알림)
pub const MAILBOX5_USER2_IRQ: u16 = 26;

/// Timer2 overflow IRQ
pub const TIMER2_IRQ: u16 = 23;

/// ePWM Trip-Zone IRQ (비상 정지)
pub const EPWM_TZ_IRQ: u16 = 33;

/// DCAN1 Line 0 IRQ (CAN 수신/송신 완료)
pub const DCAN1_LINE0_IRQ: u16 = 35;

// ═══════════════════════════════════════════════════════
// IPU Crossbar (CTRL_MODULE_CORE) — IRQ 매핑 관리
// ═══════════════════════════════════════════════════════
//
// AM5729에서 IPU Crossbar는 시스템 인터럽트 소스를 NVIC IRQ에 연결합니다.
// 기본 매핑은 TRM Table 6-53에 정의되어 있지만,
// CTRL_MODULE_CORE의 레지스터로 런타임에 변경할 수 있습니다.
//
// Crossbar 레지스터 베이스: 0x4A00_2000 (CTRL_MODULE_CORE)
// IPU1 Crossbar: 0x4A00_2A48 ~ 0x4A00_2B28 (IRQ 0~63 매핑)

/// CTRL_MODULE_CORE 베이스 (L4_CFG)
const CTRL_MODULE_CORE_BASE: u32 = 0x4A00_2000;

/// IPU1 IRQ Crossbar 레지스터 시작 오프셋
/// 각 레지스터는 16비트씩 2개의 IRQ 매핑을 포함 (packed)
/// Offset 0xA48: IRQ_0[15:0] | IRQ_1[31:16]
const IPU1_IRQ_CROSSBAR_OFFSET: u32 = 0xA48;

/// 시스템 인터럽트 번호 (Crossbar input)
pub mod sys_irq {
    /// Mailbox5 User 2 인터럽트 (IPU1용)
    pub const MAILBOX5_USER2: u16 = 255;
    /// Timer2 overflow
    pub const TIMER2_OVF: u16 = 36;
    /// ePWM0 Trip-Zone
    pub const EPWM0_TZ: u16 = 381;
    /// I2C1 인터럽트
    pub const I2C1: u16 = 56;
    /// DCAN1 인터럽트 (Line 0)
    pub const DCAN1_LINE0: u16 = 222;
    /// Reserved (Crossbar 무효 입력 — dispatcher용으로 안전)
    pub const RESERVED: u16 = 0;
}

/// Crossbar에 IRQ 매핑 설정
///
/// `nvic_irq`: NVIC IRQ 번호 (0~63)
/// `sys_irq_num`: 시스템 인터럽트 번호 (Crossbar input)
///
/// 각 레지스터는 2개의 IRQ 매핑:
///   - 짝수 IRQ: [15:0]
///   - 홀수 IRQ: [31:16]
pub fn crossbar_set_irq(nvic_irq: u16, sys_irq_num: u16) {
    unsafe {
        let reg_offset = IPU1_IRQ_CROSSBAR_OFFSET + (nvic_irq as u32 / 2) * 4;
        let reg_addr = CTRL_MODULE_CORE_BASE + reg_offset;

        let current = pac::read_reg(reg_addr);

        let new_val = if nvic_irq % 2 == 0 {
            // 하위 16비트
            (current & 0xFFFF_0000) | (sys_irq_num as u32)
        } else {
            // 상위 16비트
            (current & 0x0000_FFFF) | ((sys_irq_num as u32) << 16)
        };

        pac::write_reg(reg_addr, new_val);
    }
}

/// Crossbar에서 현재 IRQ 매핑 읽기
pub fn crossbar_get_irq(nvic_irq: u16) -> u16 {
    unsafe {
        let reg_offset = IPU1_IRQ_CROSSBAR_OFFSET + (nvic_irq as u32 / 2) * 4;
        let reg_addr = CTRL_MODULE_CORE_BASE + reg_offset;

        let val = pac::read_reg(reg_addr);

        if nvic_irq % 2 == 0 {
            (val & 0xFFFF) as u16
        } else {
            ((val >> 16) & 0xFFFF) as u16
        }
    }
}

// ═══════════════════════════════════════════════════════
// RTIC 호환 인터럽트 enum 정의
// ═══════════════════════════════════════════════════════
//
// RTIC v2는 `dispatchers = [Name, ...]`에서 enum variant를 기대합니다.
// cortex_m PAC가 아니므로, 직접 정의합니다.
//
// RTIC v2 `device = crate::pac::intc`로 사용할 수 있도록
// `Interrupt` enum과 NVIC 호환 인터페이스를 제공합니다.

/// IPU1 인터럽트 번호 — RTIC device PAC 호환
///
/// RTIC v2는 이 enum에서 dispatchers에 사용할 variant를 참조합니다.
/// `unsafe impl cortex_m::interrupt::InterruptNumber`를 구현하여
/// NVIC 조작이 가능하도록 합니다.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum Interrupt {
    // ── 실제 하드웨어 인터럽트 ──
    TIMER2 = TIMER2_IRQ,
    MAILBOX5_USER2 = MAILBOX5_USER2_IRQ,
    EPWM_TRIP_ZONE = EPWM_TZ_IRQ,
    DCAN1_LINE0 = DCAN1_LINE0_IRQ,

    // ── RTIC dispatcher용 소프트웨어 인터럽트 ──
    // 이들은 Crossbar에서 아무 소스도 연결하지 않음
    // RTIC가 NVIC_ISPR로 소프트웨어 pend 합니다
    DISPATCH_SW0 = 55,
    DISPATCH_SW1 = 56,
    DISPATCH_SW2 = 57,
    DISPATCH_SW3 = 58,
}

unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt {
    #[inline(always)]
    fn number(self) -> u16 {
        self as u16
    }
}

// ═══════════════════════════════════════════════════════
// 인터럽트 초기화
// ═══════════════════════════════════════════════════════

/// IPU1 Crossbar + NVIC 인터럽트 초기화
///
/// 1. Crossbar에 실제 하드웨어 인터럽트 매핑
/// 2. Dispatcher용 IRQ는 Crossbar RESERVED(0)으로 유지
/// 3. 실제 HW 인터럽트만 NVIC enable
///
/// 이 함수는 AMMU init 완료 후, RTIC init() 내에서 호출합니다.
/// (L4_CFG 영역 0x4A00_xxxx는 AMMU Large Page 3에서 매핑)
pub fn init_crossbar_and_nvic() {
    // ── 1. Crossbar에 실제 HW 인터럽트 소스 매핑 ──

    // Mailbox5 User2 → NVIC IRQ 26
    crossbar_set_irq(MAILBOX5_USER2_IRQ, sys_irq::MAILBOX5_USER2);

    // Timer2 overflow → NVIC IRQ 23
    crossbar_set_irq(TIMER2_IRQ, sys_irq::TIMER2_OVF);

    // ePWM Trip-Zone → NVIC IRQ 33
    crossbar_set_irq(EPWM_TZ_IRQ, sys_irq::EPWM0_TZ);

    // DCAN1 Line 0 → NVIC IRQ 35
    crossbar_set_irq(DCAN1_LINE0_IRQ, sys_irq::DCAN1_LINE0);

    // ── 2. Dispatcher IRQ들은 Crossbar에 RESERVED(0) 매핑 ──
    // 하드웨어에서 트리거되지 않도록 보장
    crossbar_set_irq(DISPATCH_SW0, sys_irq::RESERVED);
    crossbar_set_irq(DISPATCH_SW1, sys_irq::RESERVED);
    crossbar_set_irq(DISPATCH_SW2, sys_irq::RESERVED);
    crossbar_set_irq(DISPATCH_SW3, sys_irq::RESERVED);

    // ── 3. NVIC에서 HW 인터럽트 활성화 ──
    // RTIC가 dispatcher IRQ는 자동으로 활성화합니다
    unsafe {
        use cortex_m::peripheral::NVIC;

        // Mailbox5 (rpmsg 수신) — priority 2 (0x80)
        // RTIC의 priority 2와 동일하게 설정
        NVIC::unmask(Interrupt::MAILBOX5_USER2);
        // Cortex-M4 priority: 0=최고, 255=최저
        // RTIC priority N → NVIC priority (MAX-N)*step
        // AM5729 IPU1은 4비트 priority (16 levels)
        let mut nvic = cortex_m::Peripherals::steal().NVIC;
        nvic.set_priority(Interrupt::MAILBOX5_USER2, 0x80); // mid-priority

        // ePWM Trip-Zone (비상정지) — 최고 우선순위
        NVIC::unmask(Interrupt::EPWM_TRIP_ZONE);
        nvic.set_priority(Interrupt::EPWM_TRIP_ZONE, 0x10); // high-priority

        // DCAN1 (CAN 수신) — priority 3 (0x60)
        // 모터 피드백 수신은 모터 제어(priority 4)보다 낮지만
        // 피드백 전송(priority 2)보다 높게 설정
        NVIC::unmask(Interrupt::DCAN1_LINE0);
        nvic.set_priority(Interrupt::DCAN1_LINE0, 0x60);
    }
}

/// Crossbar 매핑 검증 — 디버그용
///
/// 현재 Crossbar 설정을 읽어 반환
pub fn verify_crossbar() -> [(u16, u16); 8] {
    [
        (TIMER2_IRQ, crossbar_get_irq(TIMER2_IRQ)),
        (MAILBOX5_USER2_IRQ, crossbar_get_irq(MAILBOX5_USER2_IRQ)),
        (EPWM_TZ_IRQ, crossbar_get_irq(EPWM_TZ_IRQ)),
        (DCAN1_LINE0_IRQ, crossbar_get_irq(DCAN1_LINE0_IRQ)),
        (DISPATCH_SW0, crossbar_get_irq(DISPATCH_SW0)),
        (DISPATCH_SW1, crossbar_get_irq(DISPATCH_SW1)),
        (DISPATCH_SW2, crossbar_get_irq(DISPATCH_SW2)),
        (DISPATCH_SW3, crossbar_get_irq(DISPATCH_SW3)),
    ]
}
