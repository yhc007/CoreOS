//! ─────────────────────────────────────────────────────────────
//! AM5729 DMTIMER (General Purpose Timer) 레지스터 정의
//! ─────────────────────────────────────────────────────────────
//! TRM SPRUHZ6 — Chapter 24: DMTimer

/// Timer 레지스터 오프셋
pub mod offset {
    pub const TIDR: u32 = 0x000;         // Timer ID
    pub const TIOCP_CFG: u32 = 0x010;    // Timer OCP Config
    pub const IRQ_EOI: u32 = 0x020;      // End of Interrupt
    pub const IRQSTATUS_RAW: u32 = 0x024;
    pub const IRQSTATUS: u32 = 0x028;
    pub const IRQENABLE_SET: u32 = 0x02C;
    pub const IRQENABLE_CLR: u32 = 0x030;
    pub const TCLR: u32 = 0x038;         // Timer Control (start/stop, mode)
    pub const TCRR: u32 = 0x03C;         // Timer Counter
    pub const TLDR: u32 = 0x040;         // Timer Load (auto-reload 값)
    pub const TTGR: u32 = 0x044;         // Timer Trigger (reload 트리거)
    pub const TWPS: u32 = 0x048;         // Timer Write Posting Status
    pub const TMAR: u32 = 0x04C;         // Timer Match
    pub const TCAR1: u32 = 0x050;        // Timer Capture 1
    pub const TCAR2: u32 = 0x058;        // Timer Capture 2
}

/// TCLR 레지스터 비트 필드
pub mod tclr {
    pub const ST: u32 = 1 << 0;     // Start/Stop (1=start)
    pub const AR: u32 = 1 << 1;     // Auto-Reload (1=auto)
    pub const CE: u32 = 1 << 6;     // Compare Enable
    pub const PRE: u32 = 1 << 5;    // Prescaler Enable
    // PTV[4:2] — Prescaler Timer Value
    pub const fn ptv(val: u32) -> u32 {
        (val & 0x7) << 2
    }
}

/// IRQ 비트
pub mod irq {
    pub const MAT_IT_FLAG: u32 = 1 << 0;  // Match interrupt
    pub const OVF_IT_FLAG: u32 = 1 << 1;  // Overflow interrupt
    pub const TCAR_IT_FLAG: u32 = 1 << 2; // Capture interrupt
}

/// GP Timer 드라이버
pub struct GpTimer {
    base: u32,
}

impl GpTimer {
    /// # Safety
    /// base 주소가 올바른 Timer 모듈 주소여야 합니다.
    pub const unsafe fn new(base: u32) -> Self {
        Self { base }
    }

    /// 타이머 시작 (auto-reload, 주기적 인터럽트)
    pub fn start_periodic(&self, load_value: u32) {
        unsafe {
            // 1. 타이머 정지
            super::clear_bits(self.base + offset::TCLR, tclr::ST);

            // 2. Load 값 설정 (카운터가 오버플로 → load 값으로 복원)
            super::write_reg(self.base + offset::TLDR, load_value);

            // 3. 현재 카운터도 load 값으로
            super::write_reg(self.base + offset::TCRR, load_value);

            // 4. 오버플로 인터럽트 활성화
            super::write_reg(self.base + offset::IRQENABLE_SET, irq::OVF_IT_FLAG);

            // 5. 시작 (Auto-Reload + Start)
            super::write_reg(self.base + offset::TCLR, tclr::AR | tclr::ST);
        }
    }

    /// 타이머 정지
    pub fn stop(&self) {
        unsafe {
            super::clear_bits(self.base + offset::TCLR, tclr::ST);
        }
    }

    /// 현재 카운터 값 읽기
    pub fn read_counter(&self) -> u32 {
        unsafe { super::read_reg(self.base + offset::TCRR) }
    }

    /// 인터럽트 상태 클리어
    pub fn clear_irq(&self) {
        unsafe {
            super::write_reg(self.base + offset::IRQSTATUS, irq::OVF_IT_FLAG);
        }
    }

    /// 1kHz 주기적 타이머 설정 (로봇 제어 루프용)
    /// sys_clk = 20MHz 가정, 오버플로 주기 = 1ms
    pub fn setup_1khz(&self) {
        // 20MHz 기준: 1ms = 20,000 ticks
        // load = 0xFFFFFFFF - 20000 + 1 = 0xFFFFB1E0
        let load = 0xFFFF_FFFF - 20_000 + 1;
        self.start_periodic(load);
    }
}
