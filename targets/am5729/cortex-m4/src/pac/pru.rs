//! ─────────────────────────────────────────────────────────────
//! AM5729 PRU-ICSS — Industrial Communication Subsystem
//! ─────────────────────────────────────────────────────────────
//!
//! PRU-ICSS (Programmable Real-time Unit - Industrial Communication
//! SubSystem)는 AM5729의 핵심 산업용 통신 엔진입니다.
//!
//! AM5729에는 PRU-ICSS1, PRU-ICSS2 두 개의 PRU 서브시스템이 있으며,
//! 각각 2개의 PRU 코어 (PRU0, PRU1)를 포함합니다.
//!
//! ═══════════════════════════════════════════════════════════════
//!  TSN에서 PRU-ICSS의 역할
//! ═══════════════════════════════════════════════════════════════
//!
//! CPSW가 IEEE 802.1Qbv/802.1AS 하드웨어를 제공하지만,
//! PRU-ICSS는 추가적으로:
//!
//! 1. 산업용 이더넷 프로토콜 (EtherCAT, PROFINET IRT, EtherNet/IP)
//! 2. HSR/PRP 이중화 (IEC 62439-3)
//! 3. 추가 2포트 이더넷 MAC (CPSW 외에 추가 포트)
//! 4. 사이클-정확 패킷 스케줄링 (200MHz PRU → 5ns 정밀도)
//! 5. 결정적 IPC: PRU ↔ M4 (shared memory + interrupt)
//!
//! ═══════════════════════════════════════════════════════════════
//!  Base Addresses (TRM SPRUHZ6)
//! ═══════════════════════════════════════════════════════════════
//!
//! PRU-ICSS1: 0x4B20_0000
//! PRU-ICSS2: 0x4B28_0000
//!
//! 각 PRU-ICSS 내부 메모리 맵:
//!   +0x0_0000  Data RAM 0 (8KB)   — PRU0 전용
//!   +0x0_2000  Data RAM 1 (8KB)   — PRU1 전용
//!   +0x1_0000  Shared RAM (12KB)  — PRU0/PRU1 공유
//!   +0x2_0000  INTC (Interrupt Controller)
//!   +0x2_2000  PRU0 Control
//!   +0x2_2400  PRU0 Debug
//!   +0x2_4000  PRU1 Control
//!   +0x2_4400  PRU1 Debug
//!   +0x2_6000  CFG (Configuration)
//!   +0x2_8000  UART0
//!   +0x2_C000  IEP (Industrial Ethernet Peripheral) ← TSN 타이머!
//!   +0x2_E000  eCAP
//!   +0x3_0000  MII_RT (MII Real-Time)
//!   +0x3_2000  MDIO
//!   +0x3_4000  IRAM 0 (8KB)  — PRU0 명령 메모리
//!   +0x3_8000  IRAM 1 (8KB)  — PRU1 명령 메모리

use crate::pac;

// ═══════════════════════════════════════════════════════
// Base Addresses
// ═══════════════════════════════════════════════════════

pub const PRU_ICSS1_BASE: u32 = 0x4B20_0000;
pub const PRU_ICSS2_BASE: u32 = 0x4B28_0000;

/// PRU-ICSS 내부 서브모듈 오프셋
pub const DRAM0_OFF: u32     = 0x0_0000;  // PRU0 Data RAM (8KB)
pub const DRAM1_OFF: u32     = 0x0_2000;  // PRU1 Data RAM (8KB)
pub const SHARED_RAM_OFF: u32 = 0x1_0000; // Shared RAM (12KB)
pub const INTC_OFF: u32      = 0x2_0000;  // PRU-ICSS INTC
pub const PRU0_CTRL_OFF: u32 = 0x2_2000;  // PRU0 Control
pub const PRU0_DBG_OFF: u32  = 0x2_2400;  // PRU0 Debug
pub const PRU1_CTRL_OFF: u32 = 0x2_4000;  // PRU1 Control
pub const PRU1_DBG_OFF: u32  = 0x2_4400;  // PRU1 Debug
pub const CFG_OFF: u32       = 0x2_6000;  // Configuration
pub const IEP_OFF: u32       = 0x2_C000;  // IEP (Industrial Ethernet Peripheral)
pub const MII_RT_OFF: u32    = 0x3_0000;  // MII Real-Time
pub const PRU_MDIO_OFF: u32  = 0x3_2000;  // MDIO
pub const IRAM0_OFF: u32     = 0x3_4000;  // PRU0 Instruction RAM (8KB)
pub const IRAM1_OFF: u32     = 0x3_8000;  // PRU1 Instruction RAM (8KB)

// ═══════════════════════════════════════════════════════
// IEP (Industrial Ethernet Peripheral) — TSN Timer
// ═══════════════════════════════════════════════════════
//
// IEP는 PRU-ICSS의 핵심 타이머로, TSN에서:
//   - 나노초 정밀도 타임스탬핑
//   - CPTS와 동기화하여 802.1AS gPTP 시간 유지
//   - Compare 이벤트로 정확한 사이클 타이밍 생성
//   - CMP0~7: 8개의 Compare 이벤트 (TSN GCL 타이밍)

pub mod iep {
    pub const GLOBAL_CFG: u32     = 0x00;  // Global Configuration
    pub const GLOBAL_STATUS: u32  = 0x04;  // Global Status (counter overflow)
    pub const COMPEN: u32         = 0x08;  // Compensation Enable
    pub const SLOW_COMPEN: u32    = 0x0C;  // Slow Compensation
    pub const COUNT: u32          = 0x10;  // 32-bit Counter
    pub const COUNT_HI: u32       = 0x14;  // Upper 32-bit (64-bit mode)
    pub const CAP_CFG: u32        = 0x18;  // Capture Configuration
    pub const CAP_STATUS: u32     = 0x1C;  // Capture Status

    /// Compare registers (CMP0~7)
    pub const CMP_CFG: u32      = 0x40;
    pub const CMP_STATUS: u32   = 0x44;
    pub const CMP0: u32         = 0x48;
    pub const CMP0_HI: u32      = 0x4C;
    pub const CMP1: u32         = 0x50;
    pub const CMP1_HI: u32      = 0x54;
    pub const CMP2: u32         = 0x58;
    pub const CMP3: u32         = 0x60;
    pub const CMP4: u32         = 0x68;
    pub const CMP5: u32         = 0x70;
    pub const CMP6: u32         = 0x78;
    pub const CMP7: u32         = 0x80;

    /// Sync control registers
    pub const SYNC_CTRL: u32    = 0x100;
    pub const SYNC0_PERIOD: u32 = 0x104;
    pub const SYNC0_WIDTH: u32  = 0x108;
    pub const SYNC1_PERIOD: u32 = 0x10C;
    pub const SYNC1_WIDTH: u32  = 0x110;

    /// Global Config bits
    pub const CNT_ENABLE: u32     = 1 << 0;   // Counter enable
    pub const DEFAULT_INC: u32    = 5;         // 200MHz → 5ns per tick
    pub const DEFAULT_INC_SHIFT: u32 = 4;
    pub const CMP0_RST_CNT_EN: u32 = 1 << 1;  // CMP0 resets counter

    /// Compare Config bits
    pub const CMP_EN_SHIFT: u32 = 0;  // CMP0_EN at bit 0, CMP1_EN at bit 1, etc.

    /// TSN GCL 타이밍용 Compare 할당:
    ///   CMP0: 사이클 리셋 (125μs = 25000 × 5ns)
    ///   CMP1: TC0→TC1 전환 (30μs)
    ///   CMP2: TC1→TC2 전환 (60μs)
    ///   CMP3: TC2→TC7 전환 (100μs)
    ///   CMP4: Sync pulse 생성 (모터 제어 동기화)
    pub const TSN_CYCLE_TICKS: u32 = 25_000;  // 125μs / 5ns
    pub const TSN_TC0_END: u32     = 6_000;   // 30μs / 5ns
    pub const TSN_TC1_END: u32     = 12_000;  // 60μs / 5ns
    pub const TSN_TC2_END: u32     = 20_000;  // 100μs / 5ns
}

// ═══════════════════════════════════════════════════════
// PRU Control Registers
// ═══════════════════════════════════════════════════════

pub mod pru_ctrl {
    pub const CONTROL: u32   = 0x00;
    pub const STATUS: u32    = 0x04;
    pub const WAKEUP_EN: u32 = 0x08;
    pub const CYCLE: u32     = 0x0C;
    pub const STALL: u32     = 0x10;
    pub const CTBIR0: u32    = 0x20;
    pub const CTBIR1: u32    = 0x24;
    pub const CTPPR0: u32    = 0x28;
    pub const CTPPR1: u32    = 0x2C;

    /// Control bits
    pub const SOFT_RST_N: u32  = 1 << 0;  // 0=reset, 1=run
    pub const ENABLE: u32      = 1 << 1;  // PRU enable
    pub const SLEEPING: u32    = 1 << 2;
    pub const COUNTER_EN: u32  = 1 << 3;
    pub const SINGLE_STEP: u32 = 1 << 8;
    pub const RUN_STATE: u32   = 1 << 15; // 1=running
    pub const PC_MASK: u32     = 0xFFFF0000;
    pub const PC_SHIFT: u32    = 16;
}

// ═══════════════════════════════════════════════════════
// PRU-ICSS INTC (Interrupt Controller)
// ═══════════════════════════════════════════════════════

pub mod pru_intc {
    pub const REVID: u32    = 0x00;
    pub const CR: u32       = 0x04;    // Control Register
    pub const GER: u32      = 0x10;    // Global Enable
    pub const GNLR: u32     = 0x1C;    // Global Nesting Level
    pub const SISR: u32     = 0x20;    // Sys Interrupt Status Set
    pub const SICR: u32     = 0x24;    // Sys Interrupt Status Clear
    pub const EISR: u32     = 0x28;    // Enable Interrupt Set
    pub const EICR: u32     = 0x2C;    // Enable Interrupt Clear
    pub const HIEISR: u32   = 0x34;    // Host Interrupt Enable Set
    pub const HIDISR: u32   = 0x38;    // Host Interrupt Enable Clear
    pub const SRSR0: u32    = 0x200;   // Sys Interrupt Status Raw
    pub const SECR0: u32    = 0x280;   // Sys Interrupt Status Enabled/Clear
    pub const ESR0: u32     = 0x300;   // Sys Interrupt Enable Set
    pub const CMR0: u32     = 0x400;   // Channel Map Registers
    pub const HMR0: u32     = 0x800;   // Host Map Registers

    /// Host interrupt outputs:
    ///   Host 0,1: PRU0, PRU1 내부 인터럽트
    ///   Host 2~9: 외부 출력 (MPU/M4로 전달)
    ///
    /// TSN 인터럽트 매핑:
    ///   Host 2 → M4 NVIC: IEP CMP0 (사이클 완료)
    ///   Host 3 → M4 NVIC: IEP CMP4 (모터 sync pulse)
    ///   Host 4 → M4 NVIC: MII RX complete
    pub const HOST_TSN_CYCLE: u32   = 2;
    pub const HOST_TSN_MOTOR_SYNC: u32 = 3;
    pub const HOST_TSN_RX: u32      = 4;
}

// ═══════════════════════════════════════════════════════
// MII_RT (MII Real-Time) Control
// ═══════════════════════════════════════════════════════

pub mod mii_rt {
    pub const RXCFG: u32    = 0x00;
    pub const TXCFG: u32    = 0x10;
    pub const TX_CRC: u32   = 0x20;
    pub const TX_IPG: u32   = 0x30;
    pub const PRS0: u32     = 0x40;
    pub const PRS1: u32     = 0x44;
    pub const RX_FRMS0: u32 = 0x48;
    pub const RX_FRMS1: u32 = 0x4C;

    /// RX Config bits
    pub const RX_ENABLE: u32      = 1 << 0;
    pub const RX_CUT_PREAMBLE: u32 = 1 << 2;
    pub const RX_MUX_SEL: u32     = 1 << 3;
    pub const RX_L2_EN: u32       = 1 << 4;

    /// TX Config bits
    pub const TX_ENABLE: u32      = 1 << 0;
    pub const TX_AUTO_PREAMBLE: u32 = 1 << 1;
    pub const TX_AUTO_SEQ: u32    = 1 << 4;
    pub const TX_CLK_DELAY: u32   = 1 << 16;
}

// ═══════════════════════════════════════════════════════
// Shared Memory Layout for TSN IPC (PRU ↔ M4)
// ═══════════════════════════════════════════════════════
//
// PRU Shared RAM (12KB)을 TSN 통신 버퍼로 사용:
//
// +0x0000  TSN Control Block (64 bytes)
//   - [0x00] cycle_count (u64)
//   - [0x08] current_time_ns (u64)
//   - [0x10] sync_status (u32): 0=unsync, 1=syncing, 2=synced
//   - [0x14] gm_identity (u64): gPTP Grandmaster ID
//   - [0x1C] path_delay_ns (u32): 경로 지연
//   - [0x20] freq_offset_ppb (i32): 주파수 오프셋
//
// +0x0040  TX Ring Buffer (4KB) — M4→PRU 전송 패킷
//   - Header: [write_idx, read_idx, capacity, entry_size]
//   - Entries: [timestamp(u64), len(u16), tc(u8), pad, data[128]]
//
// +0x1040  RX Ring Buffer (4KB) — PRU→M4 수신 패킷
//   - Header: [write_idx, read_idx, capacity, entry_size]
//   - Entries: [timestamp(u64), len(u16), port(u8), pad, data[128]]
//
// +0x2040  EST Schedule (256 bytes) — GCL 테이블
// +0x2140  Statistics (256 bytes) — 포트별 카운터

pub mod shared_mem {
    /// Shared RAM 내 오프셋
    pub const CTRL_BLOCK_OFF: u32   = 0x0000;
    pub const TX_RING_OFF: u32      = 0x0040;
    pub const RX_RING_OFF: u32      = 0x1040;
    pub const EST_SCHEDULE_OFF: u32 = 0x2040;
    pub const STATS_OFF: u32        = 0x2140;

    /// Control Block fields
    pub const CYCLE_COUNT_OFF: u32     = 0x00;
    pub const CURRENT_TIME_OFF: u32    = 0x08;
    pub const SYNC_STATUS_OFF: u32     = 0x10;
    pub const GM_IDENTITY_OFF: u32     = 0x14;
    pub const PATH_DELAY_OFF: u32      = 0x1C;
    pub const FREQ_OFFSET_OFF: u32     = 0x20;

    /// Sync status values
    pub const SYNC_UNSYNC: u32  = 0;
    pub const SYNC_SYNCING: u32 = 1;
    pub const SYNC_SYNCED: u32  = 2;

    /// Ring buffer header
    pub const RING_WRITE_IDX: u32  = 0x00;
    pub const RING_READ_IDX: u32   = 0x04;
    pub const RING_CAPACITY: u32   = 0x08;
    pub const RING_ENTRY_SIZE: u32 = 0x0C;
    pub const RING_HEADER_SIZE: u32 = 0x10;

    /// Ring entry layout
    pub const ENTRY_TIMESTAMP: u32 = 0x00;  // u64 (8 bytes)
    pub const ENTRY_LEN: u32       = 0x08;  // u16
    pub const ENTRY_TC_PORT: u32   = 0x0A;  // u8
    pub const ENTRY_DATA: u32      = 0x0C;  // [u8; 128]
    pub const ENTRY_SIZE: u32      = 140;   // total entry size
    pub const RING_MAX_ENTRIES: u32 = 28;   // 4096 / 144 ≈ 28
}

// ═══════════════════════════════════════════════════════
// PRU-ICSS Driver
// ═══════════════════════════════════════════════════════

/// PRU 인스턴스 선택
#[derive(Copy, Clone, Debug)]
pub enum PruInstance {
    Icss1,
    Icss2,
}

/// PRU-ICSS TSN 드라이버
pub struct PruIcss {
    base: u32,
}

impl PruIcss {
    pub const fn new(instance: PruInstance) -> Self {
        let base = match instance {
            PruInstance::Icss1 => PRU_ICSS1_BASE,
            PruInstance::Icss2 => PRU_ICSS2_BASE,
        };
        Self { base }
    }

    // ═══════════════════════════════════════════
    //  PRU 코어 제어
    // ═══════════════════════════════════════════

    /// PRU 코어 리셋
    pub fn reset_pru(&self, pru_num: u8) {
        let ctrl_base = self.base + if pru_num == 0 { PRU0_CTRL_OFF } else { PRU1_CTRL_OFF };
        unsafe {
            // Reset assert (SOFT_RST_N = 0)
            pac::clear_bits(ctrl_base + pru_ctrl::CONTROL, pru_ctrl::SOFT_RST_N);
        }
    }

    /// PRU 코어 실행 시작
    pub fn enable_pru(&self, pru_num: u8) {
        let ctrl_base = self.base + if pru_num == 0 { PRU0_CTRL_OFF } else { PRU1_CTRL_OFF };
        unsafe {
            // Enable + Release reset
            pac::write_reg(
                ctrl_base + pru_ctrl::CONTROL,
                pru_ctrl::ENABLE | pru_ctrl::SOFT_RST_N | pru_ctrl::COUNTER_EN,
            );
        }
    }

    /// PRU 코어 정지
    pub fn disable_pru(&self, pru_num: u8) {
        let ctrl_base = self.base + if pru_num == 0 { PRU0_CTRL_OFF } else { PRU1_CTRL_OFF };
        unsafe {
            pac::clear_bits(ctrl_base + pru_ctrl::CONTROL, pru_ctrl::ENABLE);
        }
    }

    /// PRU 실행 상태 확인
    pub fn is_pru_running(&self, pru_num: u8) -> bool {
        let ctrl_base = self.base + if pru_num == 0 { PRU0_CTRL_OFF } else { PRU1_CTRL_OFF };
        unsafe { pac::read_reg(ctrl_base + pru_ctrl::CONTROL) & pru_ctrl::RUN_STATE != 0 }
    }

    /// PRU 명령 메모리에 펌웨어 로드
    pub fn load_firmware(&self, pru_num: u8, firmware: &[u32]) {
        let iram_base = self.base + if pru_num == 0 { IRAM0_OFF } else { IRAM1_OFF };
        for (i, &word) in firmware.iter().enumerate() {
            unsafe {
                pac::write_reg(iram_base + (i as u32) * 4, word);
            }
        }
    }

    // ═══════════════════════════════════════════
    //  IEP (Industrial Ethernet Peripheral) Timer
    // ═══════════════════════════════════════════

    /// IEP 타이머 초기화 (TSN 125μs 사이클)
    pub fn init_iep_tsn(&self) {
        let iep_base = self.base + IEP_OFF;
        unsafe {
            // Counter 비활성화
            pac::write_reg(iep_base + iep::GLOBAL_CFG, 0);

            // Counter 초기화
            pac::write_reg(iep_base + iep::COUNT, 0);
            pac::write_reg(iep_base + iep::COUNT_HI, 0);

            // CMP0: 사이클 리셋 (125μs)
            pac::write_reg(iep_base + iep::CMP0, iep::TSN_CYCLE_TICKS);
            pac::write_reg(iep_base + iep::CMP0_HI, 0);

            // CMP1: TC0→TC1 전환점 (30μs)
            pac::write_reg(iep_base + iep::CMP1, iep::TSN_TC0_END);
            pac::write_reg(iep_base + iep::CMP1_HI, 0);

            // CMP2: TC1→TC2 전환점 (60μs)
            pac::write_reg(iep_base + iep::CMP2, iep::TSN_TC1_END);

            // CMP3: TC2→TC7 전환점 (100μs)
            pac::write_reg(iep_base + iep::CMP3, iep::TSN_TC2_END);

            // CMP4: Motor sync pulse (1ms 주기로 M4에 동기화 신호)
            pac::write_reg(iep_base + iep::CMP4, 200_000); // 1ms / 5ns

            // Compare 활성화 (CMP0~4)
            pac::write_reg(iep_base + iep::CMP_CFG, 0x1F); // bits 0~4 enable

            // Sync0: 125μs 주기 펄스 (EST GCL 동기화)
            pac::write_reg(iep_base + iep::SYNC_CTRL, 0x03);  // SYNC0 enable + cyclic
            pac::write_reg(iep_base + iep::SYNC0_PERIOD, iep::TSN_CYCLE_TICKS);
            pac::write_reg(iep_base + iep::SYNC0_WIDTH, 100); // 500ns pulse width

            // Counter 시작: 5ns increment, CMP0 auto-reset
            pac::write_reg(
                iep_base + iep::GLOBAL_CFG,
                iep::CNT_ENABLE
                    | (iep::DEFAULT_INC << iep::DEFAULT_INC_SHIFT)
                    | iep::CMP0_RST_CNT_EN,
            );
        }
    }

    /// IEP 현재 카운터 읽기 (64-bit, 5ns 단위)
    pub fn iep_get_count(&self) -> u64 {
        let iep_base = self.base + IEP_OFF;
        unsafe {
            let hi = pac::read_reg(iep_base + iep::COUNT_HI) as u64;
            let lo = pac::read_reg(iep_base + iep::COUNT) as u64;
            (hi << 32) | lo
        }
    }

    /// IEP CMP 이벤트 상태 확인 및 클리어
    pub fn iep_check_cmp(&self, cmp_num: u8) -> bool {
        let iep_base = self.base + IEP_OFF;
        let mask = 1u32 << cmp_num;
        unsafe {
            let status = pac::read_reg(iep_base + iep::CMP_STATUS);
            if status & mask != 0 {
                pac::write_reg(iep_base + iep::CMP_STATUS, mask); // W1C
                true
            } else {
                false
            }
        }
    }

    // ═══════════════════════════════════════════
    //  MII_RT 초기화
    // ═══════════════════════════════════════════

    /// MII Real-Time 모드 초기화
    pub fn init_mii_rt(&self) {
        let mii_base = self.base + MII_RT_OFF;
        unsafe {
            // RX: enable, cut preamble, L2 mode
            pac::write_reg(
                mii_base + mii_rt::RXCFG,
                mii_rt::RX_ENABLE | mii_rt::RX_CUT_PREAMBLE | mii_rt::RX_L2_EN,
            );

            // TX: enable, auto preamble
            pac::write_reg(
                mii_base + mii_rt::TXCFG,
                mii_rt::TX_ENABLE | mii_rt::TX_AUTO_PREAMBLE,
            );
        }
    }

    // ═══════════════════════════════════════════
    //  PRU-ICSS INTC 초기화
    // ═══════════════════════════════════════════

    /// PRU INTC 초기화 (TSN 인터럽트 매핑)
    pub fn init_intc_tsn(&self) {
        let intc_base = self.base + INTC_OFF;
        unsafe {
            // Global enable
            pac::write_reg(intc_base + pru_intc::GER, 1);

            // Host interrupts 2,3,4 활성화
            pac::write_reg(intc_base + pru_intc::HIEISR, pru_intc::HOST_TSN_CYCLE);
            pac::write_reg(intc_base + pru_intc::HIEISR, pru_intc::HOST_TSN_MOTOR_SYNC);
            pac::write_reg(intc_base + pru_intc::HIEISR, pru_intc::HOST_TSN_RX);
        }
    }

    // ═══════════════════════════════════════════
    //  Shared Memory TSN IPC
    // ═══════════════════════════════════════════

    /// TSN Shared Memory 초기화
    pub fn init_shared_mem_tsn(&self) {
        let shm_base = self.base + SHARED_RAM_OFF;
        unsafe {
            // Control block 초기화
            pac::write_reg(shm_base + shared_mem::CTRL_BLOCK_OFF + shared_mem::CYCLE_COUNT_OFF, 0);
            pac::write_reg(shm_base + shared_mem::CTRL_BLOCK_OFF + shared_mem::CYCLE_COUNT_OFF + 4, 0);
            pac::write_reg(shm_base + shared_mem::CTRL_BLOCK_OFF + shared_mem::SYNC_STATUS_OFF, shared_mem::SYNC_UNSYNC);

            // TX Ring buffer header
            let tx_ring = shm_base + shared_mem::TX_RING_OFF;
            pac::write_reg(tx_ring + shared_mem::RING_WRITE_IDX, 0);
            pac::write_reg(tx_ring + shared_mem::RING_READ_IDX, 0);
            pac::write_reg(tx_ring + shared_mem::RING_CAPACITY, shared_mem::RING_MAX_ENTRIES);
            pac::write_reg(tx_ring + shared_mem::RING_ENTRY_SIZE, shared_mem::ENTRY_SIZE);

            // RX Ring buffer header
            let rx_ring = shm_base + shared_mem::RX_RING_OFF;
            pac::write_reg(rx_ring + shared_mem::RING_WRITE_IDX, 0);
            pac::write_reg(rx_ring + shared_mem::RING_READ_IDX, 0);
            pac::write_reg(rx_ring + shared_mem::RING_CAPACITY, shared_mem::RING_MAX_ENTRIES);
            pac::write_reg(rx_ring + shared_mem::RING_ENTRY_SIZE, shared_mem::ENTRY_SIZE);
        }
    }

    /// gPTP 동기화 상태 읽기
    pub fn get_sync_status(&self) -> u32 {
        let shm_base = self.base + SHARED_RAM_OFF + shared_mem::CTRL_BLOCK_OFF;
        unsafe { pac::read_reg(shm_base + shared_mem::SYNC_STATUS_OFF) }
    }

    /// 현재 TSN 네트워크 시간 읽기 (nanoseconds)
    pub fn get_network_time_ns(&self) -> u64 {
        let shm_base = self.base + SHARED_RAM_OFF + shared_mem::CTRL_BLOCK_OFF;
        unsafe {
            let lo = pac::read_reg(shm_base + shared_mem::CURRENT_TIME_OFF) as u64;
            let hi = pac::read_reg(shm_base + shared_mem::CURRENT_TIME_OFF + 4) as u64;
            (hi << 32) | lo
        }
    }

    /// TSN 사이클 카운터 읽기
    pub fn get_cycle_count(&self) -> u64 {
        let shm_base = self.base + SHARED_RAM_OFF + shared_mem::CTRL_BLOCK_OFF;
        unsafe {
            let lo = pac::read_reg(shm_base + shared_mem::CYCLE_COUNT_OFF) as u64;
            let hi = pac::read_reg(shm_base + shared_mem::CYCLE_COUNT_OFF + 4) as u64;
            (hi << 32) | lo
        }
    }

    /// 경로 지연 읽기 (nanoseconds)
    pub fn get_path_delay_ns(&self) -> u32 {
        let shm_base = self.base + SHARED_RAM_OFF + shared_mem::CTRL_BLOCK_OFF;
        unsafe { pac::read_reg(shm_base + shared_mem::PATH_DELAY_OFF) }
    }

    // ═══════════════════════════════════════════
    //  전체 TSN 스택 초기화
    // ═══════════════════════════════════════════

    /// PRU-ICSS TSN 스택 전체 초기화
    ///
    /// 순서:
    /// 1. PRU0, PRU1 리셋
    /// 2. INTC 초기화 (TSN 인터럽트 매핑)
    /// 3. IEP 타이머 초기화 (125μs 사이클)
    /// 4. MII_RT 초기화 (실시간 이더넷)
    /// 5. Shared Memory 초기화 (TSN IPC 버퍼)
    /// 6. PRU 펌웨어 로드 (별도 바이너리)
    /// 7. PRU0 시작 (TX 스케줄러 + gPTP)
    /// 8. PRU1 시작 (RX 처리 + 타임스탬핑)
    pub fn init_tsn_stack(&self) {
        // 1. PRU 리셋
        self.reset_pru(0);
        self.reset_pru(1);

        // 2. INTC
        self.init_intc_tsn();

        // 3. IEP (125μs TSN 사이클)
        self.init_iep_tsn();

        // 4. MII_RT
        self.init_mii_rt();

        // 5. Shared Memory
        self.init_shared_mem_tsn();

        // 6~8. 펌웨어 로드 및 실행은 A15 Linux에서 remoteproc으로 수행
        // M4에서는 PRU Shared Memory를 통해 TSN 데이터에 접근합니다.
    }
}
