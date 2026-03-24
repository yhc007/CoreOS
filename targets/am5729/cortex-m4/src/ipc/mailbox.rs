//! ─────────────────────────────────────────────────────────────
//! AM5729 Mailbox 드라이버 (M4 측)
//! ─────────────────────────────────────────────────────────────
//!
//! Mailbox5: MPU (A15) ↔ IPU1 (M4) 통신
//!
//! 메일박스 구조:
//!   - 12개의 메일박스 슬롯 (각 FIFO 4 entries)
//!   - 4개의 유저 (IRQ 라인): USER0~3
//!   - MPU → IPU1: Mailbox 5, Sub-mailbox 4 (관례)
//!   - IPU1 → MPU: Mailbox 5, Sub-mailbox 5 (관례)
//!
//! 사용 패턴:
//!   1. A15가 Mailbox에 vring index를 write → M4에 IRQ 발생
//!   2. M4가 IRQ 핸들러에서 vring 처리
//!   3. M4가 응답 후 Mailbox에 write → A15에 IRQ 발생
//!
//! 참고: TRM SPRUHZ6 — Chapter 17: Mailbox

use crate::pac;

/// Mailbox5 레지스터 오프셋 (TRM Table 17-5)
pub mod offset {
    pub const REVISION: u32 = 0x000;
    pub const SYSCONFIG: u32 = 0x010;
    pub const MESSAGE_0: u32 = 0x040;   // Sub-mailbox 0
    pub const MESSAGE_1: u32 = 0x044;
    pub const MESSAGE_2: u32 = 0x048;
    pub const MESSAGE_3: u32 = 0x04C;
    pub const MESSAGE_4: u32 = 0x050;   // MPU → IPU1 (관례)
    pub const MESSAGE_5: u32 = 0x054;   // IPU1 → MPU (관례)
    pub const FIFOSTATUS_0: u32 = 0x080;
    pub const FIFOSTATUS_4: u32 = 0x090;
    pub const FIFOSTATUS_5: u32 = 0x094;
    pub const MSGSTATUS_0: u32 = 0x0C0;
    pub const MSGSTATUS_4: u32 = 0x0D0;
    pub const MSGSTATUS_5: u32 = 0x0D4;

    // User 2 = IPU1 인터럽트 (관례)
    pub const IRQSTATUS_RAW_2: u32 = 0x120;
    pub const IRQSTATUS_CLR_2: u32 = 0x124;
    pub const IRQENABLE_SET_2: u32 = 0x128;
    pub const IRQENABLE_CLR_2: u32 = 0x12C;
}

/// Sub-mailbox 번호
pub const MBOX_MPU_TO_IPU1: u32 = 4;  // A15 → M4
pub const MBOX_IPU1_TO_MPU: u32 = 5;  // M4 → A15

/// Mailbox 드라이버
pub struct Mailbox {
    base: u32,
}

impl Mailbox {
    /// 새 Mailbox 인스턴스 (Mailbox5 = 0x4883_A000)
    pub const unsafe fn new(base: u32) -> Self {
        Self { base }
    }

    /// Mailbox 초기화 — IPU1 수신 인터럽트 활성화
    pub fn init(&self) {
        unsafe {
            // Sub-mailbox 4 (A15→M4) 에 대한 "new message" 인터럽트 활성화
            // Bit 4 = NEWMSGENABLEUUMB_4 for User 2
            let new_msg_bit = 1u32 << (2 * MBOX_MPU_TO_IPU1); // bit for "new msg" on sub-mbox 4
            pac::write_reg(self.base + offset::IRQENABLE_SET_2, new_msg_bit);
        }
    }

    /// Sub-mailbox에서 메시지 읽기 (FIFO pop)
    /// 메시지가 없으면 None 반환
    pub fn read(&self, sub_mailbox: u32) -> Option<u32> {
        unsafe {
            let msgstatus_offset = 0x0C0 + sub_mailbox * 4;
            let count = pac::read_reg(self.base + msgstatus_offset) & 0x7;
            if count == 0 {
                return None;
            }
            let message_offset = 0x040 + sub_mailbox * 4;
            Some(pac::read_reg(self.base + message_offset))
        }
    }

    /// Sub-mailbox에 메시지 쓰기 (FIFO push)
    /// FIFO가 꽉 차면 false 반환
    pub fn write(&self, sub_mailbox: u32, msg: u32) -> bool {
        unsafe {
            let fifostatus_offset = 0x080 + sub_mailbox * 4;
            let fifo_full = pac::read_reg(self.base + fifostatus_offset) & 0x1;
            if fifo_full != 0 {
                return false;
            }
            let message_offset = 0x040 + sub_mailbox * 4;
            pac::write_reg(self.base + message_offset, msg);
            true
        }
    }

    /// A15로부터 메시지 수신 (Sub-mailbox 4)
    pub fn recv_from_a15(&self) -> Option<u32> {
        self.read(MBOX_MPU_TO_IPU1)
    }

    /// A15에 메시지 송신 (Sub-mailbox 5)
    pub fn send_to_a15(&self, msg: u32) -> bool {
        self.write(MBOX_IPU1_TO_MPU, msg)
    }

    /// IRQ 상태 확인 및 클리어
    pub fn clear_irq(&self) {
        unsafe {
            let status = pac::read_reg(self.base + offset::IRQSTATUS_RAW_2);
            // 모든 pending 인터럽트 클리어
            pac::write_reg(self.base + offset::IRQSTATUS_CLR_2, status);
        }
    }

    /// 수신 대기 중인 메시지가 있는지 확인
    pub fn has_pending_message(&self) -> bool {
        unsafe {
            let msgstatus_offset = 0x0C0 + MBOX_MPU_TO_IPU1 * 4;
            let count = pac::read_reg(self.base + msgstatus_offset) & 0x7;
            count > 0
        }
    }
}
