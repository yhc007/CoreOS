//! ─────────────────────────────────────────────────────────────
//! AM5729 DCAN (Controller Area Network) 드라이버
//! ─────────────────────────────────────────────────────────────
//!
//! TRM SPRUHZ6 — Chapter 25: DCAN
//! 로봇 모터 드라이버/센서 통신의 핵심 버스
//!
//! ═══════════════════════════════════════════════════════════════
//!  Message Object 할당 전략
//! ═══════════════════════════════════════════════════════════════
//!
//! AM5729 DCAN은 64개의 Message Object를 갖고 있습니다.
//! 각 Message Object는 TX 또는 RX 전용으로 설정됩니다.
//!
//! 할당 계획 (로봇 제어용):
//!
//! | MsgObj | 방향 | CAN ID       | 용도                    |
//! |--------|------|-------------|-------------------------|
//! |  1~8   | TX   | 0x100~0x107 | 모터 명령 (6축+예비2)    |
//! |  9~16  | TX   | 0x200~0x207 | 서보/액추에이터 명령     |
//! | 17     | TX   | 0x7FF       | Emergency Stop 브로드캐스트|
//! | 18~24  | -    | -           | 예비 (TX)               |
//! |--------|------|-------------|-------------------------|
//! | 25~32  | RX   | 0x180~0x187 | 모터 피드백 (6축+예비2)  |
//! | 33~40  | RX   | 0x280~0x287 | 센서 데이터             |
//! | 41     | RX   | 0x080       | 하트비트 (모터 드라이버) |
//! | 42     | RX   | 0x000~0x7FF | Catch-all (마스크 없음)  |
//! | 43~64  | -    | -           | 예비 (RX)               |
//!
//! ═══════════════════════════════════════════════════════════════
//!  IF1 / IF2 인터페이스
//! ═══════════════════════════════════════════════════════════════
//!
//! DCAN은 Message RAM에 직접 접근할 수 없습니다.
//! 반드시 IF1 또는 IF2 인터페이스 레지스터를 통해 간접 접근합니다.
//!
//! 관례:
//!   IF1 → TX (CPU → Message RAM 쓰기)
//!   IF2 → RX (Message RAM → CPU 읽기)
//!
//! 참고: TRM SPRUHZ6 — Chapter 25, Section 25.3.5

use crate::pac;

// ═══════════════════════════════════════════════════════
// 레지스터 오프셋
// ═══════════════════════════════════════════════════════

pub mod offset {
    pub const CTL: u32 = 0x000;          // Control
    pub const ES: u32 = 0x004;           // Error and Status
    pub const ERRC: u32 = 0x008;         // Error Counter
    pub const BTR: u32 = 0x00C;          // Bit Timing
    pub const INT: u32 = 0x010;          // Interrupt Register
    pub const TEST: u32 = 0x014;         // Test
    pub const PERR: u32 = 0x01C;         // Parity Error
    pub const ABOTR: u32 = 0x080;        // Auto-Bus-On Timer

    // Transmission Request — MsgObj가 TX 대기 중인지 확인
    pub const TXRQ_X: u32 = 0x088;       // TXRQ12 (composite)
    pub const TXRQ1: u32 = 0x098;        // TxRq MsgObj 1~32
    pub const TXRQ2: u32 = 0x09C;        // TxRq MsgObj 33~64

    // New Data — MsgObj에 새 데이터 수신 여부
    pub const NWDAT_X: u32 = 0x090;       // NWDAT12 (composite)
    pub const NWDAT1: u32 = 0x0B8;        // NewDat MsgObj 1~32
    pub const NWDAT2: u32 = 0x0BC;        // NewDat MsgObj 33~64

    // Interrupt Pending — MsgObj 인터럽트 대기
    pub const INTPND_X: u32 = 0x0A0;
    pub const INTPND1: u32 = 0x0D8;       // IntPnd MsgObj 1~32
    pub const INTPND2: u32 = 0x0DC;       // IntPnd MsgObj 33~64

    // Message Valid — MsgObj 유효 여부
    pub const MSGVAL1: u32 = 0x0F8;       // MsgVal MsgObj 1~32
    pub const MSGVAL2: u32 = 0x0FC;       // MsgVal MsgObj 33~64

    // Message Object IF1 (주로 TX용 — CPU → Message RAM)
    pub const IF1CMD: u32 = 0x100;       // IF1 Command Request
    pub const IF1MSK: u32 = 0x104;       // IF1 Mask
    pub const IF1ARB: u32 = 0x108;       // IF1 Arbitration
    pub const IF1MCTL: u32 = 0x10C;      // IF1 Message Control
    pub const IF1DATA: u32 = 0x110;      // IF1 Data A (bytes 0-3)
    pub const IF1DATB: u32 = 0x114;      // IF1 Data B (bytes 4-7)

    // Message Object IF2 (주로 RX용 — Message RAM → CPU)
    pub const IF2CMD: u32 = 0x120;
    pub const IF2MSK: u32 = 0x124;
    pub const IF2ARB: u32 = 0x128;
    pub const IF2MCTL: u32 = 0x12C;
    pub const IF2DATA: u32 = 0x130;
    pub const IF2DATB: u32 = 0x134;
}

// ═══════════════════════════════════════════════════════
// CTL 비트 필드 (TRM 25.3.3.1)
// ═══════════════════════════════════════════════════════

pub mod ctl {
    pub const INIT: u32 = 1 << 0;    // Initialization mode
    pub const IE0: u32 = 1 << 1;     // Interrupt Enable (Line 0)
    pub const SIE: u32 = 1 << 2;     // Status change Interrupt Enable
    pub const EIE: u32 = 1 << 3;     // Error Interrupt Enable
    pub const DAR: u32 = 1 << 5;     // Disable Auto Retransmission
    pub const CCE: u32 = 1 << 6;     // Configuration Change Enable
    pub const TEST_EN: u32 = 1 << 7; // Test Mode Enable
    pub const IDS: u32 = 1 << 8;     // Interrupt Debug Support
    pub const ABO: u32 = 1 << 9;     // Auto-Bus-On Enable
    pub const PMD_MASK: u32 = 0x1F << 10; // Parity on/off
    pub const SWR: u32 = 1 << 15;    // Software Reset
    pub const IE1: u32 = 1 << 17;    // Interrupt Enable (Line 1)
}

// ═══════════════════════════════════════════════════════
// IF Command 비트 필드 (TRM 25.3.5.1)
// ═══════════════════════════════════════════════════════

pub mod ifcmd {
    /// Message Number (1~64) — bits [5:0]
    pub const MSG_NUM_MASK: u32 = 0x3F;

    /// Busy flag — bit 15 (read-only, IF access in progress)
    pub const BUSY: u32 = 1 << 15;

    // Transfer Direction
    /// WR/RD = 1: Write CPU→RAM (TX setup)
    pub const WR: u32 = 1 << 7;
    /// WR/RD = 0: Read RAM→CPU (RX read)
    pub const RD: u32 = 0; // bit 7 = 0

    // Which fields to transfer
    /// Access Mask bits
    pub const MASK: u32 = 1 << 6;
    /// Access Arbitration bits
    pub const ARB: u32 = 1 << 5;
    /// Access Control bits
    pub const CONTROL: u32 = 1 << 4;
    /// Clear IntPnd bit (when reading)
    pub const CLRINTPND: u32 = 1 << 3;
    /// Set TxRqst/Clear NewDat
    pub const TXRQST_NEWDAT: u32 = 1 << 2;
    /// Access Data A (bytes 0-3)
    pub const DATA_A: u32 = 1 << 1;
    /// Access Data B (bytes 4-7)
    pub const DATA_B: u32 = 1 << 0;

    /// 전체 필드 전송 (Mask + Arb + Control + DataA + DataB)
    pub const ALL: u32 = MASK | ARB | CONTROL | DATA_A | DATA_B;
}

// ═══════════════════════════════════════════════════════
// IF Arbitration 비트 필드 (TRM 25.3.5.3)
// ═══════════════════════════════════════════════════════

pub mod ifarb {
    /// Message Valid — MsgObj 활성화
    pub const MSGVAL: u32 = 1 << 31;
    /// Extended ID 사용 (29-bit)
    pub const XTD: u32 = 1 << 30;
    /// Direction: 1 = TX, 0 = RX
    pub const DIR_TX: u32 = 1 << 29;
    pub const DIR_RX: u32 = 0; // bit 29 = 0

    /// Standard ID (11-bit) → bits [28:18]
    #[inline]
    pub const fn std_id(id: u16) -> u32 {
        ((id as u32) & 0x7FF) << 18
    }

    /// Extended ID (29-bit) → bits [28:0]
    #[inline]
    pub const fn ext_id(id: u32) -> u32 {
        id & 0x1FFF_FFFF
    }
}

// ═══════════════════════════════════════════════════════
// IF Mask 비트 필드 (TRM 25.3.5.2)
// ═══════════════════════════════════════════════════════

pub mod ifmsk {
    /// Use Mask for acceptance filtering
    pub const MXTD: u32 = 1 << 31;  // Extended ID mask bit
    pub const MDIR: u32 = 1 << 30;  // Direction mask bit

    /// Standard ID Mask → bits [28:18]
    /// 1 = 해당 비트 비교, 0 = 해당 비트 무시 (don't care)
    #[inline]
    pub const fn std_mask(mask: u16) -> u32 {
        ((mask as u32) & 0x7FF) << 18
    }

    /// Extended ID Mask → bits [28:0]
    #[inline]
    pub const fn ext_mask(mask: u32) -> u32 {
        mask & 0x1FFF_FFFF
    }

    /// 전체 11-bit Standard ID 매칭 (마스크 = 0x7FF)
    pub const STD_EXACT: u32 = 0x7FF << 18;

    /// Standard ID 매칭 없음 (모든 ID 수용)
    pub const STD_ACCEPT_ALL: u32 = 0;
}

// ═══════════════════════════════════════════════════════
// IF Message Control 비트 필드 (TRM 25.3.5.4)
// ═══════════════════════════════════════════════════════

pub mod ifmctl {
    /// DLC (Data Length Code) — bits [3:0]
    pub const DLC_MASK: u32 = 0x0F;

    /// End of Block — 1 = single msg obj (FIFO 미사용)
    pub const EOB: u32 = 1 << 7;

    /// TX Request — 이 MsgObj를 TX 큐에 넣음
    pub const TXRQST: u32 = 1 << 8;

    /// Remote Frame Enable
    pub const RMTEN: u32 = 1 << 9;

    /// RX Interrupt Enable — 수신 시 인터럽트 발생
    pub const RXIE: u32 = 1 << 10;

    /// TX Interrupt Enable — 송신 완료 시 인터럽트 발생
    pub const TXIE: u32 = 1 << 11;

    /// Use Acceptance Mask — 마스크 필터링 활성화
    pub const UMASK: u32 = 1 << 12;

    /// Interrupt Pending (read-only)
    pub const INTPND: u32 = 1 << 13;

    /// Message Lost — 수신 오버런 (read-only)
    pub const MSGLST: u32 = 1 << 14;

    /// New Data — 새 메시지 수신됨 (read-only)
    pub const NEWDAT: u32 = 1 << 15;
}

// ═══════════════════════════════════════════════════════
// Error & Status 비트 필드 (TRM 25.3.3.2)
// ═══════════════════════════════════════════════════════

pub mod es {
    /// Last Error Code (bits [2:0])
    pub const LEC_MASK: u32 = 0x07;
    pub const LEC_NO_ERROR: u32 = 0;
    pub const LEC_STUFF_ERROR: u32 = 1;
    pub const LEC_FORM_ERROR: u32 = 2;
    pub const LEC_ACK_ERROR: u32 = 3;
    pub const LEC_BIT1_ERROR: u32 = 4;
    pub const LEC_BIT0_ERROR: u32 = 5;
    pub const LEC_CRC_ERROR: u32 = 6;

    /// TX OK — 메시지 성공적으로 전송됨
    pub const TXOK: u32 = 1 << 3;
    /// RX OK — 메시지 성공적으로 수신됨
    pub const RXOK: u32 = 1 << 4;
    /// Error Passive
    pub const EPASS: u32 = 1 << 5;
    /// Warning level reached (TEC/REC ≥ 96)
    pub const EWARN: u32 = 1 << 6;
    /// Bus Off
    pub const BOFF: u32 = 1 << 7;
    /// Parity Error Detected
    pub const PER: u32 = 1 << 8;
}

// ═══════════════════════════════════════════════════════
// CAN 메시지 구조체
// ═══════════════════════════════════════════════════════

/// 수신된 CAN 프레임
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct CanFrame {
    /// Standard ID (11-bit) 또는 Extended ID (29-bit)
    pub id: u32,
    /// Extended ID 여부
    pub is_extended: bool,
    /// 데이터 길이 (0~8)
    pub dlc: u8,
    /// 페이로드 (최대 8바이트)
    pub data: [u8; 8],
    /// 수신한 Message Object 번호 (1~64)
    pub msg_obj: u8,
    /// Message Lost 플래그 (수신 오버런 발생 여부)
    pub msg_lost: bool,
}

impl CanFrame {
    pub const fn empty() -> Self {
        Self {
            id: 0,
            is_extended: false,
            dlc: 0,
            data: [0; 8],
            msg_obj: 0,
            msg_lost: false,
        }
    }
}

// ═══════════════════════════════════════════════════════
// 미리 정의된 CAN ID 상수 (로봇 프로토콜)
// ═══════════════════════════════════════════════════════

pub mod can_id {
    // ── TX (M4 → 외부 디바이스) ──
    /// 모터 명령 베이스 ID (0x100 + motor_id)
    pub const MOTOR_CMD_BASE: u16 = 0x100;
    /// 서보 명령 베이스 ID
    pub const SERVO_CMD_BASE: u16 = 0x200;
    /// Emergency Stop 브로드캐스트
    pub const EMERGENCY_STOP: u16 = 0x7FF;

    // ── RX (외부 디바이스 → M4) ──
    /// 모터 피드백 베이스 ID (0x180 + motor_id)
    pub const MOTOR_FB_BASE: u16 = 0x180;
    /// 센서 데이터 베이스 ID (0x280 + sensor_id)
    pub const SENSOR_DATA_BASE: u16 = 0x280;
    /// 모터 드라이버 하트비트
    pub const HEARTBEAT: u16 = 0x080;
}

// ═══════════════════════════════════════════════════════
// Message Object 번호 할당
// ═══════════════════════════════════════════════════════

pub mod msg_obj {
    // TX Message Objects (1~24)
    pub const TX_MOTOR_BASE: u8 = 1;    // 1~8: 모터 명령
    pub const TX_SERVO_BASE: u8 = 9;    // 9~16: 서보 명령
    pub const TX_ESTOP: u8 = 17;        // 17: Emergency Stop

    // RX Message Objects (25~64)
    pub const RX_MOTOR_FB_BASE: u8 = 25; // 25~32: 모터 피드백
    pub const RX_SENSOR_BASE: u8 = 33;   // 33~40: 센서 데이터
    pub const RX_HEARTBEAT: u8 = 41;     // 41: 하트비트
    pub const RX_CATCH_ALL: u8 = 42;     // 42: Catch-all (디버그용)
}

// ═══════════════════════════════════════════════════════
// DCAN 드라이버
// ═══════════════════════════════════════════════════════

pub struct Dcan {
    base: u32,
}

impl Dcan {
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
    // IF Busy 대기
    // ═══════════════════════════════════════════════════

    /// IF1 (TX) 인터페이스 대기
    fn wait_if1_ready(&self) {
        let mut timeout = 10_000u32;
        while self.read(offset::IF1CMD) & ifcmd::BUSY != 0 {
            timeout -= 1;
            if timeout == 0 { break; }
        }
    }

    /// IF2 (RX) 인터페이스 대기
    fn wait_if2_ready(&self) {
        let mut timeout = 10_000u32;
        while self.read(offset::IF2CMD) & ifcmd::BUSY != 0 {
            timeout -= 1;
            if timeout == 0 { break; }
        }
    }

    // ═══════════════════════════════════════════════════
    // 초기화
    // ═══════════════════════════════════════════════════

    /// CAN 초기화 (1Mbps @ 20MHz DCAN 클럭)
    ///
    /// Bit Timing:
    ///   BRP=0 → prescaler=1 → tq = 1/20MHz = 50ns
    ///   TSEG1=14, TSEG2=4, SJW=2
    ///   Nominal bit time = (1+14+5) * 50ns = 1.0μs → 1Mbps
    ///   Sample point = (1+14)/20 = 75%
    pub fn init_1mbps(&self) {
        // 1. Init 모드 진입 + Configuration Change Enable
        unsafe { pac::set_bits(self.base + offset::CTL, ctl::INIT | ctl::CCE); }

        // 2. Bit Timing 설정
        let btr = (0 << 0)    // BRP = 0 (prescaler = 1)
                | (14 << 8)   // TSEG1 = 14
                | (4 << 12)   // TSEG2 = 4
                | (2 << 6);   // SJW = 2
        self.write(offset::BTR, btr);

        // 3. 모든 64개 Message Object 무효화
        self.invalidate_all_msg_objects();

        // 4. 인터럽트 활성화 (Line 0 + Status + Error)
        unsafe {
            pac::set_bits(self.base + offset::CTL,
                ctl::IE0 | ctl::SIE | ctl::EIE | ctl::ABO);
        }

        // 5. Init 모드 해제 → 정상 동작
        unsafe { pac::clear_bits(self.base + offset::CTL, ctl::INIT | ctl::CCE); }
    }

    /// CAN 500kbps 초기화 (모터 드라이버 호환)
    ///
    /// BRP=1 → prescaler=2 → tq = 100ns
    /// (1+14+5) * 100ns = 2.0μs → 500kbps
    pub fn init_500kbps(&self) {
        unsafe { pac::set_bits(self.base + offset::CTL, ctl::INIT | ctl::CCE); }

        let btr = (1 << 0)    // BRP = 1 (prescaler = 2)
                | (14 << 8)   // TSEG1 = 14
                | (4 << 12)   // TSEG2 = 4
                | (2 << 6);   // SJW = 2
        self.write(offset::BTR, btr);

        self.invalidate_all_msg_objects();

        unsafe {
            pac::set_bits(self.base + offset::CTL,
                ctl::IE0 | ctl::SIE | ctl::EIE | ctl::ABO);
            pac::clear_bits(self.base + offset::CTL, ctl::INIT | ctl::CCE);
        }
    }

    /// 모든 Message Object 무효화 (초기화 시)
    fn invalidate_all_msg_objects(&self) {
        for obj in 1..=64u8 {
            self.wait_if1_ready();
            // Arbitration: MsgVal = 0 (무효)
            self.write(offset::IF1ARB, 0);
            self.write(offset::IF1MCTL, 0);
            // IF1CMD: Write + Arb + Control 전송
            self.write(offset::IF1CMD,
                ifcmd::WR | ifcmd::ARB | ifcmd::CONTROL | (obj as u32));
        }
    }

    // ═══════════════════════════════════════════════════
    // RX Message Object 설정
    // ═══════════════════════════════════════════════════

    /// Standard ID 수신 Message Object 설정
    ///
    /// `msg_obj`: Message Object 번호 (1~64)
    /// `id`: 매칭할 Standard CAN ID (11-bit)
    /// `mask`: ID 마스크 (1=비교, 0=무시)
    /// `use_irq`: 수신 인터럽트 활성화 여부
    ///
    /// ```
    /// // MsgObj 25: ID 0x180 정확 매칭, 인터럽트 활성화
    /// dcan.setup_rx_std(25, 0x180, 0x7FF, true);
    ///
    /// // MsgObj 33: ID 0x280~0x287 범위 수신 (하위 3비트 무시)
    /// dcan.setup_rx_std(33, 0x280, 0x7F8, true);
    ///
    /// // MsgObj 42: 모든 ID 수신 (catch-all, 마스크=0)
    /// dcan.setup_rx_std(42, 0x000, 0x000, false);
    /// ```
    pub fn setup_rx_std(&self, msg_obj: u8, id: u16, mask: u16, use_irq: bool) {
        assert!(msg_obj >= 1 && msg_obj <= 64);

        self.wait_if1_ready();

        // Mask: Standard ID 필터 마스크 + UMASK에 의해 활성화
        let msk = ifmsk::std_mask(mask) | ifmsk::MDIR; // Direction 도 체크
        self.write(offset::IF1MSK, msk);

        // Arbitration: MsgVal + RX 방향 + Standard ID
        let arb = ifarb::MSGVAL | ifarb::DIR_RX | ifarb::std_id(id);
        self.write(offset::IF1ARB, arb);

        // Message Control: DLC=8 + EOB + UMASK + 옵션 RXIE
        let mut mctl = 8u32             // DLC = 8 (최대 수신)
                     | ifmctl::EOB      // End of Block (단일 MsgObj)
                     | ifmctl::UMASK;   // Use Acceptance Mask
        if use_irq {
            mctl |= ifmctl::RXIE;      // RX 인터럽트 활성화
        }
        self.write(offset::IF1MCTL, mctl);

        // Data 초기화 (0으로)
        self.write(offset::IF1DATA, 0);
        self.write(offset::IF1DATB, 0);

        // IF1CMD: Write ALL fields to Message RAM
        self.write(offset::IF1CMD,
            ifcmd::WR | ifcmd::ALL | (msg_obj as u32));
    }

    /// Extended ID (29-bit) 수신 Message Object 설정
    pub fn setup_rx_ext(&self, msg_obj: u8, id: u32, mask: u32, use_irq: bool) {
        assert!(msg_obj >= 1 && msg_obj <= 64);

        self.wait_if1_ready();

        let msk = ifmsk::ext_mask(mask) | ifmsk::MXTD | ifmsk::MDIR;
        self.write(offset::IF1MSK, msk);

        let arb = ifarb::MSGVAL | ifarb::XTD | ifarb::DIR_RX | ifarb::ext_id(id);
        self.write(offset::IF1ARB, arb);

        let mut mctl = 8u32 | ifmctl::EOB | ifmctl::UMASK;
        if use_irq {
            mctl |= ifmctl::RXIE;
        }
        self.write(offset::IF1MCTL, mctl);

        self.write(offset::IF1DATA, 0);
        self.write(offset::IF1DATB, 0);

        self.write(offset::IF1CMD,
            ifcmd::WR | ifcmd::ALL | (msg_obj as u32));
    }

    /// 로봇 제어용 전체 RX Message Object 일괄 설정
    ///
    /// 미리 정의된 할당 전략에 따라 모터 피드백, 센서, 하트비트,
    /// catch-all 수신 오브젝트를 한번에 설정합니다.
    pub fn setup_robot_rx_objects(&self) {
        // ── 모터 피드백 (MsgObj 25~32, ID 0x180~0x187) ──
        // 각 모터 드라이버가 0x180+motor_id로 피드백 전송
        for i in 0..8u8 {
            self.setup_rx_std(
                msg_obj::RX_MOTOR_FB_BASE + i,
                can_id::MOTOR_FB_BASE + i as u16,
                0x7FF,  // 정확 매칭
                true,   // 인터럽트 ON
            );
        }

        // ── 센서 데이터 (MsgObj 33~40, ID 0x280~0x287) ──
        for i in 0..8u8 {
            self.setup_rx_std(
                msg_obj::RX_SENSOR_BASE + i,
                can_id::SENSOR_DATA_BASE + i as u16,
                0x7FF,
                true,
            );
        }

        // ── 하트비트 (MsgObj 41, ID 0x080) ──
        self.setup_rx_std(
            msg_obj::RX_HEARTBEAT,
            can_id::HEARTBEAT,
            0x7FF,
            false,  // 인터럽트 OFF (폴링으로 확인)
        );

        // ── Catch-all (MsgObj 42, 모든 ID 수용) ──
        // 디버그 및 알 수 없는 메시지 캡처용
        self.setup_rx_std(
            msg_obj::RX_CATCH_ALL,
            0x000,
            0x000,  // 마스크 없음 → 모든 ID 수용
            false,
        );
    }

    // ═══════════════════════════════════════════════════
    // RX 메시지 읽기
    // ═══════════════════════════════════════════════════

    /// Message Object에서 수신 데이터 읽기
    ///
    /// NewData 플래그가 설정된 MsgObj에서 데이터를 읽습니다.
    /// 읽은 후 NewData와 IntPnd가 자동으로 클리어됩니다.
    ///
    /// ```
    /// if let Some(frame) = dcan.read_msg_obj(25) {
    ///     info!("Motor 0 feedback: ID=0x{:03X}, data={}", frame.id, frame.data);
    /// }
    /// ```
    pub fn read_msg_obj(&self, msg_obj: u8) -> Option<CanFrame> {
        assert!(msg_obj >= 1 && msg_obj <= 64);

        // NewData 확인
        if !self.has_new_data(msg_obj) {
            return None;
        }

        self.wait_if2_ready();

        // IF2CMD: Read + ALL fields + ClearIntPnd + ClearNewDat
        self.write(offset::IF2CMD,
            ifcmd::RD
            | ifcmd::ALL
            | ifcmd::CLRINTPND
            | ifcmd::TXRQST_NEWDAT  // NewDat 클리어
            | (msg_obj as u32));

        self.wait_if2_ready();

        // Arbitration 읽기
        let arb = self.read(offset::IF2ARB);
        let is_extended = arb & ifarb::XTD != 0;
        let id = if is_extended {
            arb & 0x1FFF_FFFF
        } else {
            (arb >> 18) & 0x7FF
        };

        // Message Control 읽기
        let mctl = self.read(offset::IF2MCTL);
        let dlc = (mctl & ifmctl::DLC_MASK) as u8;
        let msg_lost = mctl & ifmctl::MSGLST != 0;

        // Data 읽기
        let data_a = self.read(offset::IF2DATA);
        let data_b = self.read(offset::IF2DATB);

        let mut data = [0u8; 8];
        data[0] = (data_a & 0xFF) as u8;
        data[1] = ((data_a >> 8) & 0xFF) as u8;
        data[2] = ((data_a >> 16) & 0xFF) as u8;
        data[3] = ((data_a >> 24) & 0xFF) as u8;
        data[4] = (data_b & 0xFF) as u8;
        data[5] = ((data_b >> 8) & 0xFF) as u8;
        data[6] = ((data_b >> 16) & 0xFF) as u8;
        data[7] = ((data_b >> 24) & 0xFF) as u8;

        // MsgLst 클리어 (있었다면)
        if msg_lost {
            self.clear_msg_lost(msg_obj);
        }

        Some(CanFrame {
            id,
            is_extended,
            dlc: dlc.min(8),
            data,
            msg_obj,
            msg_lost,
        })
    }

    /// 인터럽트 레지스터에서 pending MsgObj 번호 읽기
    ///
    /// INT 레지스터 [15:0]에 가장 높은 우선순위의 pending MsgObj 번호 반환
    /// (0이면 pending 없음, 0x8000이면 Status Interrupt)
    pub fn read_int_id(&self) -> u16 {
        (self.read(offset::INT) & 0xFFFF) as u16
    }

    /// 수신 인터럽트 처리 — pending된 모든 RX MsgObj 읽기
    ///
    /// DCAN 인터럽트 핸들러에서 호출합니다.
    /// 최대 `max_frames`개까지 읽어서 버퍼에 저장합니다.
    ///
    /// ```
    /// let mut frames = [CanFrame::empty(); 8];
    /// let count = dcan.process_rx_interrupts(&mut frames);
    /// for i in 0..count {
    ///     handle_can_frame(&frames[i]);
    /// }
    /// ```
    pub fn process_rx_interrupts(&self, buf: &mut [CanFrame]) -> usize {
        let mut count = 0;

        // 모든 pending 인터럽트 처리 (최대 buf 크기까지)
        loop {
            if count >= buf.len() {
                break;
            }

            let int_id = self.read_int_id();
            if int_id == 0 {
                break; // 더 이상 pending 없음
            }
            if int_id == 0x8000 {
                // Status Interrupt — ES 레지스터 읽어서 클리어
                let _status = self.read(offset::ES);
                continue;
            }
            if int_id > 64 {
                break; // 유효하지 않은 값
            }

            let msg_obj = int_id as u8;
            if let Some(frame) = self.read_msg_obj(msg_obj) {
                buf[count] = frame;
                count += 1;
            }
        }

        count
    }

    // ═══════════════════════════════════════════════════
    // TX 메시지 송신
    // ═══════════════════════════════════════════════════

    /// CAN 메시지 송신 (Standard ID)
    ///
    /// ```
    /// // Motor 0에 속도 명령 전송
    /// dcan.send_std(1, 0x100, &[0x00, 0x64], 2); // 100 RPM
    ///
    /// // Emergency Stop 브로드캐스트
    /// dcan.send_std(17, 0x7FF, &[0xE0], 1);
    /// ```
    pub fn send_std(&self, msg_obj: u8, id: u16, data: &[u8], dlc: u8) {
        assert!(msg_obj >= 1 && msg_obj <= 64);
        assert!(dlc <= 8);

        self.wait_if1_ready();

        // Mask: TX에서는 사용하지 않음
        self.write(offset::IF1MSK, 0);

        // Arbitration: MsgVal + TX + Standard ID
        let arb = ifarb::MSGVAL | ifarb::DIR_TX | ifarb::std_id(id);
        self.write(offset::IF1ARB, arb);

        // Message Control: DLC + TxRqst + EOB + TXIE
        let mctl = (dlc as u32)
                 | ifmctl::TXRQST
                 | ifmctl::EOB
                 | ifmctl::TXIE;
        self.write(offset::IF1MCTL, mctl);

        // Data A (bytes 0-3)
        let mut data_a: u32 = 0;
        for (i, &b) in data.iter().take(4).enumerate() {
            data_a |= (b as u32) << (i * 8);
        }
        self.write(offset::IF1DATA, data_a);

        // Data B (bytes 4-7)
        let mut data_b: u32 = 0;
        for (i, &b) in data.iter().skip(4).take(4).enumerate() {
            data_b |= (b as u32) << (i * 8);
        }
        self.write(offset::IF1DATB, data_b);

        // IF1CMD: Write ALL to Message RAM
        self.write(offset::IF1CMD,
            ifcmd::WR | ifcmd::ALL | (msg_obj as u32));
    }

    /// CAN 메시지 송신 (Extended ID, 29-bit)
    pub fn send_ext(&self, msg_obj: u8, id: u32, data: &[u8], dlc: u8) {
        assert!(msg_obj >= 1 && msg_obj <= 64);
        assert!(dlc <= 8);

        self.wait_if1_ready();

        self.write(offset::IF1MSK, 0);

        let arb = ifarb::MSGVAL | ifarb::XTD | ifarb::DIR_TX | ifarb::ext_id(id);
        self.write(offset::IF1ARB, arb);

        let mctl = (dlc as u32) | ifmctl::TXRQST | ifmctl::EOB | ifmctl::TXIE;
        self.write(offset::IF1MCTL, mctl);

        let mut data_a: u32 = 0;
        for (i, &b) in data.iter().take(4).enumerate() {
            data_a |= (b as u32) << (i * 8);
        }
        self.write(offset::IF1DATA, data_a);

        let mut data_b: u32 = 0;
        for (i, &b) in data.iter().skip(4).take(4).enumerate() {
            data_b |= (b as u32) << (i * 8);
        }
        self.write(offset::IF1DATB, data_b);

        self.write(offset::IF1CMD,
            ifcmd::WR | ifcmd::ALL | (msg_obj as u32));
    }

    /// 이전 호환 send() (send_std 래퍼)
    pub fn send(&self, msg_obj: u8, id: u32, data: &[u8], dlc: u8) {
        self.send_std(msg_obj, id as u16, data, dlc);
    }

    // ═══════════════════════════════════════════════════
    // 상태 확인
    // ═══════════════════════════════════════════════════

    /// 특정 MsgObj에 새 데이터가 있는지 확인
    pub fn has_new_data(&self, msg_obj: u8) -> bool {
        assert!(msg_obj >= 1 && msg_obj <= 64);

        if msg_obj <= 32 {
            let bit = 1u32 << (msg_obj - 1);
            self.read(offset::NWDAT1) & bit != 0
        } else {
            let bit = 1u32 << (msg_obj - 33);
            self.read(offset::NWDAT2) & bit != 0
        }
    }

    /// 수신 대기 중인 MsgObj 비트맵 반환 (1~32)
    pub fn new_data_bitmap_low(&self) -> u32 {
        self.read(offset::NWDAT1)
    }

    /// 수신 대기 중인 MsgObj 비트맵 반환 (33~64)
    pub fn new_data_bitmap_high(&self) -> u32 {
        self.read(offset::NWDAT2)
    }

    /// 인터럽트 pending MsgObj 비트맵 (1~32)
    pub fn int_pending_low(&self) -> u32 {
        self.read(offset::INTPND1)
    }

    /// 인터럽트 pending MsgObj 비트맵 (33~64)
    pub fn int_pending_high(&self) -> u32 {
        self.read(offset::INTPND2)
    }

    /// Message Lost 클리어 (수신 오버런 플래그)
    fn clear_msg_lost(&self, msg_obj: u8) {
        self.wait_if1_ready();

        // MCTL에서 MSGLST 비트만 클리어
        let mctl = self.read(offset::IF1MCTL) & !ifmctl::MSGLST;
        self.write(offset::IF1MCTL, mctl);

        self.write(offset::IF1CMD,
            ifcmd::WR | ifcmd::CONTROL | (msg_obj as u32));
    }

    /// 에러 카운터 읽기 (TX, RX)
    pub fn error_counters(&self) -> (u8, u8) {
        let errc = self.read(offset::ERRC);
        let tx_err = (errc & 0xFF) as u8;
        let rx_err = ((errc >> 8) & 0x7F) as u8;
        (tx_err, rx_err)
    }

    /// Error & Status 레지스터 읽기
    pub fn error_status(&self) -> u32 {
        self.read(offset::ES)
    }

    /// Bus Off 상태인지 확인
    pub fn is_bus_off(&self) -> bool {
        self.read(offset::ES) & es::BOFF != 0
    }

    /// Error Passive 상태인지 확인
    pub fn is_error_passive(&self) -> bool {
        self.read(offset::ES) & es::EPASS != 0
    }

    /// 마지막 에러 코드 읽기
    pub fn last_error_code(&self) -> u32 {
        self.read(offset::ES) & es::LEC_MASK
    }

    /// DCAN 모듈 소프트 리셋
    pub fn soft_reset(&self) {
        unsafe { pac::set_bits(self.base + offset::CTL, ctl::SWR); }
        // 리셋 완료 대기
        let mut timeout = 10_000u32;
        while self.read(offset::CTL) & ctl::SWR != 0 {
            timeout -= 1;
            if timeout == 0 { break; }
        }
    }
}
