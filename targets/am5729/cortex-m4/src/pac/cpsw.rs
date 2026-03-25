//! ─────────────────────────────────────────────────────────────
//! AM5729 CPSW (Common Platform Switch) — TSN Ethernet Driver
//! ─────────────────────────────────────────────────────────────
//!
//! AM5729 GMAC_SW (CPSW) 이더넷 스위치의 TSN 기능을 제어합니다.
//!
//! ═══════════════════════════════════════════════════════════════
//!  AM5729 CPSW 구조
//! ═══════════════════════════════════════════════════════════════
//!
//! CPSW는 3-port Ethernet Switch로 구성:
//!   Port 0: Host (CPU) port — DMA를 통해 패킷 송수신
//!   Port 1: RGMII → External PHY 1 (1Gbps)
//!   Port 2: RGMII → External PHY 2 (1Gbps)
//!
//! TSN 관련 하드웨어 블록:
//!   - CPTS (Common Platform Time Sync): IEEE 1588 / 802.1AS 타임스탬핑
//!   - ALE (Address Lookup Engine): VLAN, 멀티캐스트 필터링
//!   - EST (Enhanced Scheduled Traffic): 802.1Qbv 타임-어웨어 셰이퍼
//!   - IET (Interspersing Express Traffic): 802.1Qbu 프레임 프리엠션
//!
//! ═══════════════════════════════════════════════════════════════
//!  Base Addresses (TRM SPRUHZ6 Table 2-3)
//! ═══════════════════════════════════════════════════════════════
//!
//! CPSW는 L4_PER3 도메인에 위치:
//!   GMAC_SW: 0x4848_0000 (AMMU Large Page 1 → L4_PER 매핑)
//!
//! 주요 서브모듈 오프셋:
//!   CPSW_SS      : +0x0_0000  (Switch Subsystem)
//!   CPSW_PORT    : +0x0_0100  (Port Control)
//!   CPSW_CPDMA   : +0x0_0800  (DMA Engine)
//!   CPSW_STATS   : +0x0_0900  (Statistics)
//!   CPSW_STATERAM: +0x0_0A00  (State RAM)
//!   CPSW_CPTS    : +0x0_0C00  (Time Sync)
//!   CPSW_ALE     : +0x0_0D00  (Address Lookup)
//!   CPSW_SL1     : +0x0_0D80  (Sliver/MAC Port 1)
//!   CPSW_SL2     : +0x0_0DC0  (Sliver/MAC Port 2)
//!   CPSW_WR      : +0x0_1200  (Wrapper)
//!   MDIO         : +0x0_1000  (MDIO for PHY Management)

use crate::pac;

// ═══════════════════════════════════════════════════════
// Base Addresses
// ═══════════════════════════════════════════════════════

/// CPSW Base Address (GMAC_SW on L4_PER3)
pub const CPSW_BASE: u32 = 0x4848_0000;

/// Sub-module offsets
pub const CPSW_SS_OFF: u32      = 0x0000;
pub const CPSW_PORT_OFF: u32    = 0x0100;
pub const CPSW_CPDMA_OFF: u32   = 0x0800;
pub const CPSW_STATS_OFF: u32   = 0x0900;
pub const CPSW_CPTS_OFF: u32    = 0x0C00;
pub const CPSW_ALE_OFF: u32     = 0x0D00;
pub const CPSW_SL1_OFF: u32     = 0x0D80;
pub const CPSW_SL2_OFF: u32     = 0x0DC0;
pub const CPSW_WR_OFF: u32      = 0x1200;
pub const MDIO_OFF: u32         = 0x1000;

// ═══════════════════════════════════════════════════════
// CPTS (Common Platform Time Sync) Registers
// ═══════════════════════════════════════════════════════
//
// CPTS는 IEEE 1588v2 / IEEE 802.1AS (gPTP) 하드웨어 타임스탬핑을 제공합니다.
// 나노초 정밀도의 64비트 타임스탬프로 패킷 송수신 시각을 기록합니다.

pub mod cpts {
    /// CPTS register offsets (from CPSW_CPTS base)
    pub const IDVER: u32           = 0x00;   // Identification/Version
    pub const CONTROL: u32         = 0x04;   // CPTS Control
    pub const RFTCLK_SEL: u32      = 0x08;   // Reference Clock Select
    pub const TS_PUSH: u32         = 0x0C;   // Timestamp Push
    pub const TS_LOAD_VAL: u32     = 0x10;   // Timestamp Load Low
    pub const TS_LOAD_EN: u32      = 0x14;   // Timestamp Load Enable
    pub const TS_COMP_VAL: u32     = 0x18;   // Timestamp Compare Low
    pub const TS_COMP_LEN: u32     = 0x1C;   // Timestamp Compare Length
    pub const INTSTAT_RAW: u32     = 0x20;   // Interrupt Status Raw
    pub const INTSTAT_MASKED: u32  = 0x24;   // Interrupt Status Masked
    pub const INT_ENABLE: u32      = 0x28;   // Interrupt Enable
    pub const TS_COMP_NUDGE: u32   = 0x2C;   // Timestamp Compare Nudge
    pub const EVENT_POP: u32       = 0x30;   // Event Pop
    pub const EVENT_LOW: u32       = 0x34;   // Event Low Timestamp
    pub const EVENT_HIGH: u32      = 0x38;   // Event High Timestamp
    pub const EVENT_INFO: u32      = 0x3C;   // Event Info (type, port, seq)

    /// CPTS Control Register bits
    pub const CPTS_EN: u32         = 1 << 0;   // CPTS 활성화
    pub const INT_TEST: u32        = 1 << 1;   // Interrupt test mode
    pub const TS_COMP_POLARITY: u32 = 1 << 2;  // Timestamp compare output polarity
    pub const HW_TS_PUSH_EN: u32   = 1 << 3;   // HW timestamp push enable
    pub const TS_COMP_TOG: u32     = 1 << 4;   // Timestamp compare toggle mode
    pub const TS_PPM_DIR: u32      = 1 << 5;   // PPM direction (0=add, 1=sub)
    pub const TS_RX_EN: u32        = 1 << 8;   // RX timestamping enable
    pub const TS_64BIT_MODE: u32   = 1 << 10;  // 64-bit timestamp mode

    /// CPTS Event Types (from EVENT_INFO register)
    pub const EVENT_TYPE_SHIFT: u32 = 0;
    pub const EVENT_TYPE_MASK: u32  = 0xF;
    pub const EVENT_TS_PUSH: u32    = 0;  // Timestamp Push event
    pub const EVENT_TIME_STAMP: u32 = 1;  // Ethernet timestamp event
    pub const EVENT_TS_COMP: u32    = 2;  // Timestamp Compare event
    pub const EVENT_HOST_TX: u32    = 3;  // Host TX timestamp
    pub const EVENT_RX: u32         = 4;  // RX timestamp

    /// EVENT_INFO field extraction
    pub const EVENT_PORT_SHIFT: u32    = 24;
    pub const EVENT_PORT_MASK: u32     = 0x1F;
    pub const EVENT_SEQ_ID_SHIFT: u32  = 0;
    pub const EVENT_SEQ_ID_MASK: u32   = 0xFFFF;
    pub const EVENT_MSG_TYPE_SHIFT: u32 = 16;
    pub const EVENT_MSG_TYPE_MASK: u32  = 0xF;
}

// ═══════════════════════════════════════════════════════
// ALE (Address Lookup Engine) Registers
// ═══════════════════════════════════════════════════════
//
// ALE은 L2 패킷 필터링, VLAN 처리, 멀티캐스트 관리를 담당합니다.
// TSN에서는 gPTP 멀티캐스트(01:80:C2:00:00:0E) 필터링과
// VLAN 기반 트래픽 분리에 사용됩니다.

pub mod ale {
    pub const IDVER: u32      = 0x00;
    pub const CONTROL: u32    = 0x08;
    pub const PRESCALE: u32   = 0x10;
    pub const UNKNOWN_VLAN: u32 = 0x18;
    pub const TBLCTL: u32     = 0x20;
    pub const TBLW2: u32      = 0x34;
    pub const TBLW1: u32      = 0x38;
    pub const TBLW0: u32      = 0x3C;
    pub const PORTCTL0: u32   = 0x40;
    pub const PORTCTL1: u32   = 0x44;
    pub const PORTCTL2: u32   = 0x48;

    /// ALE Control bits
    pub const ENABLE_ALE: u32     = 1 << 31;
    pub const CLEAR_TABLE: u32    = 1 << 30;
    pub const EN_AUTH_MODE: u32   = 1 << 22;
    pub const ENABLE_BYPASS: u32  = 1 << 4;
    pub const ENABLE_RATE_LIMIT: u32 = 1 << 2;
    pub const ENABLE_OUI_DENY: u32 = 1 << 5;

    /// ALE Table Control
    pub const WRITE_RDZ: u32    = 1 << 31;
    pub const ENTRY_MASK: u32   = 0x3FF;

    /// ALE Port Control
    pub const PORT_STATE_SHIFT: u32 = 0;
    pub const PORT_STATE_MASK: u32  = 0x3;
    pub const PORT_DISABLED: u32    = 0;
    pub const PORT_BLOCKED: u32     = 1;
    pub const PORT_LEARNING: u32    = 2;
    pub const PORT_FORWARD: u32     = 3;

    /// TSN gPTP Multicast Address: 01:80:C2:00:00:0E
    pub const GPTP_MCAST_ADDR: [u8; 6] = [0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E];
}

// ═══════════════════════════════════════════════════════
// CPDMA (DMA Engine) Registers
// ═══════════════════════════════════════════════════════

pub mod cpdma {
    pub const TX_IDVER: u32      = 0x00;
    pub const TX_CONTROL: u32    = 0x04;
    pub const TX_TEARDOWN: u32   = 0x08;
    pub const RX_IDVER: u32      = 0x10;
    pub const RX_CONTROL: u32    = 0x14;
    pub const RX_TEARDOWN: u32   = 0x18;
    pub const SOFT_RESET: u32    = 0x1C;
    pub const DMACONTROL: u32    = 0x20;
    pub const DMASTATUS: u32     = 0x24;
    pub const RX_BUFFER_OFFSET: u32 = 0x28;

    /// TX Channel Head Descriptor Pointer (channels 0~7)
    pub const TX_HDP_BASE: u32 = 0x100;
    /// RX Channel Head Descriptor Pointer (channels 0~7)
    pub const RX_HDP_BASE: u32 = 0x120;
    /// TX Completion Pointer (channels 0~7)
    pub const TX_CP_BASE: u32  = 0x140;
    /// RX Completion Pointer (channels 0~7)
    pub const RX_CP_BASE: u32  = 0x160;

    /// DMA Control bits
    pub const TX_PTYPE: u32 = 1 << 0;   // TX priority type (1=fixed)
    pub const RX_OWNERSHIP: u32 = 1 << 1;
    pub const CMD_IDLE: u32 = 1 << 3;

    /// TX Channel priority for TSN (channel 0 = highest = RT traffic)
    pub const TSN_RT_CHANNEL: u32 = 0;    // 모터 제어 패킷
    pub const TSN_AVB_CHANNEL: u32 = 1;   // 센서 스트림
    pub const TSN_BE_CHANNEL: u32 = 7;    // Best-Effort 트래픽
}

// ═══════════════════════════════════════════════════════
// Sliver (MAC Port) Registers
// ═══════════════════════════════════════════════════════

pub mod sliver {
    pub const IDVER: u32         = 0x00;
    pub const MACCONTROL: u32    = 0x04;
    pub const MACSTATUS: u32     = 0x08;
    pub const SOFT_RESET: u32    = 0x0C;
    pub const RX_MAXLEN: u32     = 0x10;
    pub const BOFFTEST: u32      = 0x14;
    pub const RX_PAUSE: u32      = 0x18;
    pub const TX_PAUSE: u32      = 0x1C;
    pub const EMCONTROL: u32     = 0x20;

    /// MAC Control bits
    pub const FULLDUPLEX: u32      = 1 << 0;
    pub const LOOPBACK: u32        = 1 << 1;
    pub const MTEST: u32           = 1 << 2;
    pub const RX_FLOW_EN: u32      = 1 << 3;
    pub const TX_FLOW_EN: u32      = 1 << 4;
    pub const GMII_EN: u32         = 1 << 5;
    pub const TX_PACE: u32         = 1 << 6;
    pub const GIG: u32             = 1 << 7;
    pub const TX_SHORT_GAP_EN: u32 = 1 << 10;
    pub const CMD_IDLE: u32        = 1 << 11;
    pub const IFCTL_A: u32         = 1 << 15;
    pub const IFCTL_B: u32         = 1 << 16;
    pub const GIG_FORCE: u32       = 1 << 17;
    pub const EXT_EN: u32          = 1 << 18;
}

// ═══════════════════════════════════════════════════════
// MDIO Registers (PHY Management)
// ═══════════════════════════════════════════════════════

pub mod mdio {
    pub const VER: u32        = 0x00;
    pub const CONTROL: u32    = 0x04;
    pub const ALIVE: u32      = 0x08;
    pub const LINK: u32       = 0x0C;
    pub const LINKINTRAW: u32 = 0x10;
    pub const LINKINTMASKED: u32 = 0x14;
    pub const USERACCESS0: u32 = 0x80;
    pub const USERPHYSEL0: u32 = 0x84;
    pub const USERACCESS1: u32 = 0x88;
    pub const USERPHYSEL1: u32 = 0x8C;

    /// MDIO Control
    pub const ENABLE: u32       = 1 << 30;
    pub const PREAMBLE: u32     = 1 << 20;
    pub const CLKDIV_MASK: u32  = 0xFFFF;

    /// User Access
    pub const GO: u32          = 1 << 31;
    pub const WRITE: u32       = 1 << 30;
    pub const ACK: u32         = 1 << 29;
    pub const REGADR_SHIFT: u32 = 21;
    pub const PHYADR_SHIFT: u32 = 16;
    pub const DATA_MASK: u32   = 0xFFFF;
}

// ═══════════════════════════════════════════════════════
// EST (Enhanced Scheduled Traffic) — IEEE 802.1Qbv
// ═══════════════════════════════════════════════════════
//
// EST는 시간 기반 트래픽 스케줄링을 제공합니다.
// Gate Control List(GCL)로 각 타임슬롯에서 어떤 큐(TC)를 열지 결정합니다.
//
// TSN 사이클 예시 (125μs cycle):
//   [0-30μs]   TC0 OPEN → 모터 제어 패킷 (hard RT)
//   [30-60μs]  TC1 OPEN → 센서 데이터 (scheduled)
//   [60-100μs] TC2 OPEN → 비전/CBCT 데이터 (AVB)
//   [100-125μs] TC7 OPEN → Best-Effort (진단, 로깅)

pub mod est {
    /// EST Control register (port-specific, in CPSW_PORT area)
    pub const EST_CONTROL_OFF: u32 = 0x64;

    /// EST control bits
    pub const EST_ENABLE: u32    = 1 << 0;   // EST 활성화
    pub const EST_ONEBUF: u32    = 1 << 1;   // Single buffer mode
    pub const EST_ONEPRI: u32    = 1 << 2;   // Single priority mode

    /// GCL (Gate Control List) entry
    /// 각 엔트리: [31:20] interval (ns), [7:0] gate mask (TC0~TC7)
    pub const GCL_INTERVAL_SHIFT: u32 = 20;
    pub const GCL_GATE_MASK: u32      = 0xFF;

    /// Traffic Class (TC) gate bits
    pub const TC0_OPEN: u8 = 1 << 0;  // Motor control (highest priority)
    pub const TC1_OPEN: u8 = 1 << 1;  // Sensor data
    pub const TC2_OPEN: u8 = 1 << 2;  // Vision/CBCT streaming
    pub const TC3_OPEN: u8 = 1 << 3;  // Reserved
    pub const TC4_OPEN: u8 = 1 << 4;  // Reserved
    pub const TC5_OPEN: u8 = 1 << 5;  // gPTP sync
    pub const TC6_OPEN: u8 = 1 << 6;  // Safety messages
    pub const TC7_OPEN: u8 = 1 << 7;  // Best-Effort

    /// 로봇 제어용 GCL 프리셋
    ///
    /// 125μs cycle (8kHz):
    ///   TC0: 30μs → 모터 제어 (24% bandwidth)
    ///   TC1: 30μs → 센서 피드백 (24%)
    ///   TC2: 40μs → 비전/벌크 (32%)
    ///   TC7: 25μs → Best-Effort (20%)
    pub const ROBOT_GCL_ENTRIES: u32 = 4;
    pub const ROBOT_CYCLE_NS: u32 = 125_000;  // 125μs = 8kHz

    pub const ROBOT_GCL: [(u32, u8); 4] = [
        (30_000, TC0_OPEN),   // 0-30μs: motor control
        (30_000, TC1_OPEN),   // 30-60μs: sensor data
        (40_000, TC2_OPEN | TC5_OPEN),  // 60-100μs: vision + gPTP
        (25_000, TC7_OPEN | TC6_OPEN),  // 100-125μs: BE + safety
    ];
}

// ═══════════════════════════════════════════════════════
// TSN Frame Definitions
// ═══════════════════════════════════════════════════════

/// gPTP (802.1AS) 메시지 타입
pub mod gptp {
    pub const SYNC: u8           = 0x00;
    pub const FOLLOW_UP: u8      = 0x08;
    pub const PDELAY_REQ: u8     = 0x02;
    pub const PDELAY_RESP: u8    = 0x03;
    pub const PDELAY_RESP_FU: u8 = 0x0A;
    pub const ANNOUNCE: u8       = 0x0B;

    /// gPTP EtherType
    pub const ETHERTYPE: u16 = 0x88F7;

    /// gPTP Multicast destination
    pub const MCAST_ADDR: [u8; 6] = [0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E];
}

// ═══════════════════════════════════════════════════════
// CPSW Driver
// ═══════════════════════════════════════════════════════

/// CPTS 타임스탬프 (64-bit, nanoseconds)
#[derive(Copy, Clone, Debug, Default)]
pub struct CptsTimestamp {
    pub seconds: u32,
    pub nanoseconds: u32,
}

/// CPTS 이벤트 (타임스탬프 + 메타데이터)
#[derive(Copy, Clone, Debug)]
pub struct CptsEvent {
    pub timestamp: CptsTimestamp,
    pub event_type: u8,
    pub port_num: u8,
    pub msg_type: u8,
    pub seq_id: u16,
}

/// DMA Buffer Descriptor (CPPI 3.0)
#[repr(C, align(16))]
#[derive(Copy, Clone)]
pub struct BufferDescriptor {
    pub next_desc: u32,
    pub buf_ptr: u32,
    pub buf_off_len: u32,
    pub pkt_flags_len: u32,
}

/// CPSW Ethernet Switch Driver
pub struct Cpsw {
    base: u32,
}

impl Cpsw {
    /// 새 CPSW 드라이버 인스턴스 생성
    pub const fn new() -> Self {
        Self { base: CPSW_BASE }
    }

    // ═══════════════════════════════════════════
    //  초기화
    // ═══════════════════════════════════════════

    /// CPSW 소프트 리셋
    pub fn soft_reset(&self) {
        let wr_base = self.base + CPSW_WR_OFF;
        unsafe {
            pac::write_reg(wr_base + 0x04, 1); // SOFT_RESET
        }
        // Wait for reset complete
        for _ in 0..10000 {
            let val = unsafe { pac::read_reg(wr_base + 0x04) };
            if val & 1 == 0 { return; }
        }
    }

    /// TSN 모드로 CPSW 초기화
    ///
    /// 1. 소프트 리셋
    /// 2. ALE 초기화 (VLAN + gPTP 멀티캐스트)
    /// 3. CPTS 초기화 (802.1AS 타임스탬핑)
    /// 4. MAC 포트 설정 (1Gbps Full-Duplex)
    /// 5. MDIO 초기화 (PHY 관리)
    /// 6. CPDMA 초기화
    /// 7. EST 설정 (802.1Qbv 스케줄링)
    pub fn init_tsn(&self) {
        // 1. Soft reset
        self.soft_reset();

        // 2. ALE 초기화
        self.init_ale();

        // 3. CPTS 초기화 (802.1AS)
        self.init_cpts();

        // 4. MAC Port 1 설정 (1Gbps, Full-Duplex, RGMII)
        self.init_mac_port(1);
        self.init_mac_port(2);

        // 5. MDIO 초기화
        self.init_mdio();

        // 6. CPDMA 초기화
        self.init_cpdma();

        // 7. EST (802.1Qbv) 스케줄링 설정
        self.setup_est_robot();
    }

    // ═══════════════════════════════════════════
    //  ALE (Address Lookup Engine)
    // ═══════════════════════════════════════════

    fn init_ale(&self) {
        let ale_base = self.base + CPSW_ALE_OFF;
        unsafe {
            // ALE 활성화 + 테이블 초기화
            pac::write_reg(ale_base + ale::CONTROL, ale::ENABLE_ALE | ale::CLEAR_TABLE);

            // 잠시 대기 후 clear bit 해제
            for _ in 0..1000 { core::hint::spin_loop(); }
            pac::write_reg(ale_base + ale::CONTROL, ale::ENABLE_ALE);

            // Port 0 (Host): Forward 상태
            pac::write_reg(ale_base + ale::PORTCTL0, ale::PORT_FORWARD);
            // Port 1: Forward 상태
            pac::write_reg(ale_base + ale::PORTCTL1, ale::PORT_FORWARD);
            // Port 2: Forward 상태
            pac::write_reg(ale_base + ale::PORTCTL2, ale::PORT_FORWARD);

            // gPTP 멀티캐스트 주소 등록 (01:80:C2:00:00:0E → Host port로 전달)
            self.ale_add_mcast(&ale::GPTP_MCAST_ADDR, 0x01); // Port 0 (host) only
        }
    }

    /// ALE 테이블에 멀티캐스트 주소 추가
    fn ale_add_mcast(&self, addr: &[u8; 6], port_mask: u8) {
        let ale_base = self.base + CPSW_ALE_OFF;
        unsafe {
            // ALE entry format for multicast:
            // Word 0: MAC[1:0] (byte 4,5)
            // Word 1: MAC[3:2] (byte 2,3) | entry_type[61:60] | mcast_fwd_state
            // Word 2: port_mask | MAC[5:4] (byte 0,1)

            let w0: u32 = (addr[4] as u32) << 8 | (addr[5] as u32);
            let w1: u32 = (addr[0] as u32) << 8 | (addr[1] as u32)
                         | (addr[2] as u32) << 24 | (addr[3] as u32) << 16
                         | (0x1 << 28); // entry_type = multicast (01)
            let w2: u32 = (port_mask as u32) << 2; // super bit + port_mask

            // 빈 엔트리 찾기 (간단히 첫 번째 사용)
            pac::write_reg(ale_base + ale::TBLW0, w0);
            pac::write_reg(ale_base + ale::TBLW1, w1);
            pac::write_reg(ale_base + ale::TBLW2, w2);
            pac::write_reg(ale_base + ale::TBLCTL, ale::WRITE_RDZ | 0); // entry 0
        }
    }

    // ═══════════════════════════════════════════
    //  CPTS (802.1AS / gPTP Time Sync)
    // ═══════════════════════════════════════════

    fn init_cpts(&self) {
        let cpts_base = self.base + CPSW_CPTS_OFF;
        unsafe {
            // CPTS 활성화: 64비트 모드, RX 타임스탬핑, HW 타임스탬프 푸시
            pac::write_reg(
                cpts_base + cpts::CONTROL,
                cpts::CPTS_EN | cpts::TS_64BIT_MODE | cpts::TS_RX_EN | cpts::HW_TS_PUSH_EN,
            );

            // Reference Clock 선택 (DPLL_GMAC → 250MHz)
            pac::write_reg(cpts_base + cpts::RFTCLK_SEL, 0);

            // 인터럽트 활성화 (TS event push)
            pac::write_reg(cpts_base + cpts::INT_ENABLE, 0x01);

            // 초기 타임스탬프 로드 (0으로 시작, gPTP sync 후 보정)
            pac::write_reg(cpts_base + cpts::TS_LOAD_VAL, 0);
            pac::write_reg(cpts_base + cpts::TS_LOAD_EN, 1);
        }
    }

    /// CPTS 현재 시간 읽기 (64-bit timestamp)
    ///
    /// TS_PUSH를 트리거하여 현재 시간을 이벤트 FIFO에 캡처한 후 읽습니다.
    pub fn cpts_get_time(&self) -> CptsTimestamp {
        let cpts_base = self.base + CPSW_CPTS_OFF;
        unsafe {
            // Push current time to event FIFO
            pac::write_reg(cpts_base + cpts::TS_PUSH, 1);

            // Wait for event (간단한 폴링)
            for _ in 0..10000 {
                let raw = pac::read_reg(cpts_base + cpts::INTSTAT_RAW);
                if raw & 0x01 != 0 { break; }
            }

            // Read event
            let low = pac::read_reg(cpts_base + cpts::EVENT_LOW);
            let high = pac::read_reg(cpts_base + cpts::EVENT_HIGH);

            // Pop event
            pac::write_reg(cpts_base + cpts::EVENT_POP, 1);

            CptsTimestamp {
                seconds: high,
                nanoseconds: low,
            }
        }
    }

    /// CPTS 이벤트 FIFO에서 타임스탬프 이벤트 읽기
    ///
    /// gPTP Sync/Follow_Up/PDelay 패킷의 HW 타임스탬프를 가져옵니다.
    pub fn cpts_pop_event(&self) -> Option<CptsEvent> {
        let cpts_base = self.base + CPSW_CPTS_OFF;
        unsafe {
            let raw = pac::read_reg(cpts_base + cpts::INTSTAT_RAW);
            if raw & 0x01 == 0 {
                return None;
            }

            let low = pac::read_reg(cpts_base + cpts::EVENT_LOW);
            let high = pac::read_reg(cpts_base + cpts::EVENT_HIGH);
            let info = pac::read_reg(cpts_base + cpts::EVENT_INFO);

            // Pop event
            pac::write_reg(cpts_base + cpts::EVENT_POP, 1);

            let event_type = ((info >> cpts::EVENT_TYPE_SHIFT) & cpts::EVENT_TYPE_MASK) as u8;
            let port_num = ((info >> cpts::EVENT_PORT_SHIFT) & cpts::EVENT_PORT_MASK) as u8;
            let msg_type = ((info >> cpts::EVENT_MSG_TYPE_SHIFT) & cpts::EVENT_MSG_TYPE_MASK) as u8;
            let seq_id = ((info >> cpts::EVENT_SEQ_ID_SHIFT) & cpts::EVENT_SEQ_ID_MASK) as u16;

            Some(CptsEvent {
                timestamp: CptsTimestamp { seconds: high, nanoseconds: low },
                event_type,
                port_num,
                msg_type,
                seq_id,
            })
        }
    }

    /// CPTS 시간 보정 (gPTP Sync 수신 시 호출)
    ///
    /// master_time: gPTP grandmaster의 현재 시간
    /// correction: 경로 지연 보정값 (ns)
    pub fn cpts_adjust_time(&self, offset_ns: i64) {
        let cpts_base = self.base + CPSW_CPTS_OFF;
        unsafe {
            if offset_ns >= 0 {
                // 양수 보정: PPM direction = add
                pac::clear_bits(cpts_base + cpts::CONTROL, cpts::TS_PPM_DIR);
            } else {
                // 음수 보정: PPM direction = subtract
                pac::set_bits(cpts_base + cpts::CONTROL, cpts::TS_PPM_DIR);
            }

            // Compare nudge로 미세 보정
            let nudge = (offset_ns.unsigned_abs() & 0xFF) as u32;
            pac::write_reg(cpts_base + cpts::TS_COMP_NUDGE, nudge);
        }
    }

    // ═══════════════════════════════════════════
    //  MAC Port 초기화
    // ═══════════════════════════════════════════

    fn init_mac_port(&self, port: u8) {
        let sl_base = self.base + if port == 1 { CPSW_SL1_OFF } else { CPSW_SL2_OFF };
        unsafe {
            // Soft reset
            pac::write_reg(sl_base + sliver::SOFT_RESET, 1);
            for _ in 0..10000 {
                if pac::read_reg(sl_base + sliver::SOFT_RESET) & 1 == 0 { break; }
            }

            // 1Gbps Full-Duplex RGMII
            pac::write_reg(
                sl_base + sliver::MACCONTROL,
                sliver::FULLDUPLEX | sliver::GMII_EN | sliver::GIG | sliver::GIG_FORCE
                | sliver::RX_FLOW_EN | sliver::TX_FLOW_EN,
            );

            // Max frame size (TSN = standard 1518 + VLAN tag)
            pac::write_reg(sl_base + sliver::RX_MAXLEN, 1522);
        }
    }

    // ═══════════════════════════════════════════
    //  MDIO 초기화
    // ═══════════════════════════════════════════

    fn init_mdio(&self) {
        let mdio_base = self.base + MDIO_OFF;
        unsafe {
            // MDIO 활성화, Clock = SYSCLK / (CLKDIV+1)
            // AM5729 SYSCLK=250MHz, MDIO max=2.5MHz → CLKDIV=99
            pac::write_reg(
                mdio_base + mdio::CONTROL,
                mdio::ENABLE | mdio::PREAMBLE | 99,
            );
        }
    }

    /// MDIO를 통해 PHY 레지스터 읽기
    pub fn mdio_read(&self, phy_addr: u8, reg_addr: u8) -> u16 {
        let mdio_base = self.base + MDIO_OFF;
        unsafe {
            let cmd = mdio::GO
                | ((reg_addr as u32) << mdio::REGADR_SHIFT)
                | ((phy_addr as u32) << mdio::PHYADR_SHIFT);
            pac::write_reg(mdio_base + mdio::USERACCESS0, cmd);

            // Wait for completion
            for _ in 0..10000 {
                let val = pac::read_reg(mdio_base + mdio::USERACCESS0);
                if val & mdio::GO == 0 {
                    if val & mdio::ACK != 0 {
                        return (val & mdio::DATA_MASK) as u16;
                    }
                    return 0;
                }
            }
            0
        }
    }

    /// MDIO를 통해 PHY 레지스터 쓰기
    pub fn mdio_write(&self, phy_addr: u8, reg_addr: u8, data: u16) {
        let mdio_base = self.base + MDIO_OFF;
        unsafe {
            let cmd = mdio::GO | mdio::WRITE
                | ((reg_addr as u32) << mdio::REGADR_SHIFT)
                | ((phy_addr as u32) << mdio::PHYADR_SHIFT)
                | (data as u32);
            pac::write_reg(mdio_base + mdio::USERACCESS0, cmd);

            for _ in 0..10000 {
                if pac::read_reg(mdio_base + mdio::USERACCESS0) & mdio::GO == 0 { break; }
            }
        }
    }

    /// PHY 링크 상태 확인
    pub fn phy_link_status(&self) -> u32 {
        let mdio_base = self.base + MDIO_OFF;
        unsafe { pac::read_reg(mdio_base + mdio::LINK) }
    }

    // ═══════════════════════════════════════════
    //  CPDMA 초기화
    // ═══════════════════════════════════════════

    fn init_cpdma(&self) {
        let dma_base = self.base + CPSW_CPDMA_OFF;
        unsafe {
            // Soft reset
            pac::write_reg(dma_base + cpdma::SOFT_RESET, 1);
            for _ in 0..10000 {
                if pac::read_reg(dma_base + cpdma::SOFT_RESET) & 1 == 0 { break; }
            }

            // TX/RX 활성화
            pac::write_reg(dma_base + cpdma::TX_CONTROL, 1);
            pac::write_reg(dma_base + cpdma::RX_CONTROL, 1);

            // DMA Control: fixed priority TX
            pac::write_reg(dma_base + cpdma::DMACONTROL, cpdma::TX_PTYPE);
        }
    }

    // ═══════════════════════════════════════════
    //  EST (802.1Qbv) — Time-Aware Shaper
    // ═══════════════════════════════════════════

    /// 로봇 제어용 EST 스케줄링 설정
    ///
    /// 125μs 사이클로 4개 타임슬롯에 트래픽 클래스를 할당합니다:
    ///   TC0(30μs): 모터 제어 — 결정적 저지연
    ///   TC1(30μs): 센서 피드백 — 주기적
    ///   TC2+TC5(40μs): 비전 데이터 + gPTP — 대역폭 확보
    ///   TC7+TC6(25μs): Best-Effort + 안전 메시지
    pub fn setup_est_robot(&self) {
        let port_base = self.base + CPSW_PORT_OFF;
        unsafe {
            // EST 비활성화 (설정 변경 전)
            pac::clear_bits(port_base + est::EST_CONTROL_OFF, est::EST_ENABLE);

            // GCL 엔트리 프로그래밍
            // (실제 하드웨어에서는 EST fetch RAM에 직접 기록)
            // 여기서는 Port EST Control을 통해 기본 설정
            for (i, (interval_ns, gate_mask)) in est::ROBOT_GCL.iter().enumerate() {
                let gcl_entry = ((*interval_ns >> 3) << est::GCL_INTERVAL_SHIFT)
                    | (*gate_mask as u32 & est::GCL_GATE_MASK);
                // GCL RAM offset (port-specific)
                let gcl_addr = port_base + 0x80 + (i as u32) * 4;
                pac::write_reg(gcl_addr, gcl_entry);
            }

            // EST 활성화
            pac::set_bits(port_base + est::EST_CONTROL_OFF, est::EST_ENABLE);
        }
    }

    /// EST 사이클 상태 확인
    pub fn est_is_active(&self) -> bool {
        let port_base = self.base + CPSW_PORT_OFF;
        unsafe {
            pac::read_reg(port_base + est::EST_CONTROL_OFF) & est::EST_ENABLE != 0
        }
    }
}
