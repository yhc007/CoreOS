//! ─────────────────────────────────────────────────────────────
//! rpmsg 프로토콜 구현 (M4 측)
//! ─────────────────────────────────────────────────────────────
//!
//! Linux rpmsg 프레임워크와 호환되는 메시지 포맷 사용.
//!
//! rpmsg 메시지 헤더 (rpmsg_hdr):
//!   ┌──────────────────────────────┐
//!   │  src: u32   — 송신 엔드포인트  │
//!   │  dst: u32   — 수신 엔드포인트  │
//!   │  reserved: u32               │
//!   │  len: u16   — 페이로드 길이    │
//!   │  flags: u16                  │
//!   │  data: [u8; N] — 페이로드     │
//!   └──────────────────────────────┘
//!
//! Name Service Announcement (NS):
//!   M4가 엔드포인트를 생성하면, A15에 NS 메시지를 보내서
//!   채널 이름을 알립니다. Linux 커널의 rpmsg_virtio가 이를
//!   처리하여 /dev/rpmsg0 등의 디바이스를 생성합니다.

use super::mailbox::{self, Mailbox, MBOX_IPU1_TO_MPU, MBOX_MPU_TO_IPU1};
use super::vring::{Vring, RPMSG_BUF_SIZE, VRING_SIZE};
use crate::pac;

/// rpmsg 메시지 헤더 (16 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct RpmsgHdr {
    pub src: u32,
    pub dst: u32,
    pub reserved: u32,
    pub len: u16,
    pub flags: u16,
}

/// Name Service Announcement (NS) 메시지
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct RpmsgNsMsg {
    /// 채널 이름 (최대 32 bytes, null-terminated)
    pub name: [u8; 32],
    /// 로컬 엔드포인트 주소
    pub addr: u32,
    /// 플래그 (RPMSG_NS_CREATE=0, RPMSG_NS_DESTROY=1)
    pub flags: u32,
}

/// NS 메시지 플래그
pub const RPMSG_NS_CREATE: u32 = 0;
pub const RPMSG_NS_DESTROY: u32 = 1;

/// rpmsg 예약 엔드포인트
pub const RPMSG_NS_ADDR: u32 = 53;           // Name Service 엔드포인트
pub const RPMSG_ROBOT_CTRL_ADDR: u32 = 100;  // 로봇 제어 엔드포인트
pub const RPMSG_ROBOT_FB_ADDR: u32 = 101;    // 로봇 피드백 엔드포인트
pub const RPMSG_DEBUG_ADDR: u32 = 200;       // 디버그 엔드포인트

/// rpmsg 엔드포인트
pub struct RpmsgEndpoint {
    pub local_addr: u32,
    pub remote_addr: u32,
    pub name: [u8; 32],
}

impl RpmsgEndpoint {
    pub fn new(name: &str, local_addr: u32) -> Self {
        let mut name_buf = [0u8; 32];
        let bytes = name.as_bytes();
        let copy_len = bytes.len().min(31);
        name_buf[..copy_len].copy_from_slice(&bytes[..copy_len]);

        Self {
            local_addr,
            remote_addr: 0xFFFF_FFFF, // 아직 바인드 안됨
            name: name_buf,
        }
    }
}

/// rpmsg 채널 (Mailbox + 2x Vring 통합)
pub struct RpmsgChannel {
    pub mailbox: Mailbox,
    pub rx_vring: Vring,  // A15 → M4 (vring0)
    pub tx_vring: Vring,  // M4 → A15 (vring1)
    /// TX 버퍼 풀에서 다음 사용할 인덱스
    tx_buf_idx: u16,
    /// 초기화 완료 여부
    initialized: bool,
}

impl RpmsgChannel {
    /// rpmsg 채널 생성
    ///
    /// `rx_vring_addr`: Resource table에 정의된 RX vring 물리 주소
    /// `tx_vring_addr`: Resource table에 정의된 TX vring 물리 주소
    pub fn new(rx_vring_addr: u32, tx_vring_addr: u32) -> Self {
        Self {
            mailbox: unsafe { Mailbox::new(pac::addr::MAILBOX5) },
            rx_vring: Vring::new(rx_vring_addr, VRING_SIZE as u16),
            tx_vring: Vring::new(tx_vring_addr, VRING_SIZE as u16),
            tx_buf_idx: 0,
            initialized: false,
        }
    }

    /// rpmsg 초기화 (remoteproc이 리소스 테이블 파싱 후 호출)
    pub fn init(&mut self) {
        // Mailbox 인터럽트 활성화
        self.mailbox.init();
        self.initialized = true;
    }

    /// Name Service 어나운스먼트 전송
    /// A15의 rpmsg_virtio에 새 엔드포인트를 알림
    pub fn announce_endpoint(&mut self, ep: &RpmsgEndpoint) {
        let ns_msg = RpmsgNsMsg {
            name: ep.name,
            addr: ep.local_addr,
            flags: RPMSG_NS_CREATE,
        };

        let ns_bytes = unsafe {
            core::slice::from_raw_parts(
                &ns_msg as *const _ as *const u8,
                core::mem::size_of::<RpmsgNsMsg>(),
            )
        };

        self.send_raw(ep.local_addr, RPMSG_NS_ADDR, ns_bytes);
    }

    /// rpmsg 메시지 수신 (Mailbox IRQ 핸들러에서 호출)
    ///
    /// 반환: (src, dst, payload_ptr, payload_len) or None
    pub fn recv(&mut self) -> Option<(u32, u32, u32, u16)> {
        // 1. Mailbox에서 vring index 읽기
        if let Some(_vring_idx) = self.mailbox.recv_from_a15() {
            // 2. RX vring에서 available descriptor 가져오기
            if let Some(desc_idx) = self.rx_vring.get_available() {
                let (buf_addr, _buf_len, _flags) = self.rx_vring.get_desc(desc_idx);

                // 3. rpmsg 헤더 읽기
                unsafe {
                    let hdr = core::ptr::read_volatile(buf_addr as *const RpmsgHdr);
                    let payload_addr = buf_addr + core::mem::size_of::<RpmsgHdr>() as u32;

                    // 4. Used ring에 반환 (버퍼 재활용)
                    self.rx_vring.add_used(desc_idx, RPMSG_BUF_SIZE as u32);

                    return Some((hdr.src, hdr.dst, payload_addr, hdr.len));
                }
            }
        }
        None
    }

    /// rpmsg 메시지에서 페이로드 복사
    pub fn copy_payload(payload_addr: u32, buf: &mut [u8], len: u16) -> usize {
        let copy_len = (len as usize).min(buf.len());
        unsafe {
            core::ptr::copy_nonoverlapping(
                payload_addr as *const u8,
                buf.as_mut_ptr(),
                copy_len,
            );
        }
        copy_len
    }

    /// rpmsg 메시지 송신
    pub fn send(&mut self, src: u32, dst: u32, data: &[u8]) -> bool {
        self.send_raw(src, dst, data)
    }

    /// 내부 raw 송신
    fn send_raw(&mut self, src: u32, dst: u32, data: &[u8]) -> bool {
        if data.len() > RPMSG_BUF_SIZE - core::mem::size_of::<RpmsgHdr>() {
            return false;
        }

        // TX vring에서 빈 버퍼 할당
        let desc_idx = self.tx_buf_idx;
        self.tx_buf_idx = (self.tx_buf_idx + 1) % VRING_SIZE as u16;

        let (buf_addr, _buf_len, _flags) = self.tx_vring.get_desc(desc_idx);

        unsafe {
            // rpmsg 헤더 쓰기
            let hdr = RpmsgHdr {
                src,
                dst,
                reserved: 0,
                len: data.len() as u16,
                flags: 0,
            };
            core::ptr::write_volatile(buf_addr as *mut RpmsgHdr, hdr);

            // 페이로드 쓰기
            let payload_addr = buf_addr + core::mem::size_of::<RpmsgHdr>() as u32;
            core::ptr::copy_nonoverlapping(
                data.as_ptr(),
                payload_addr as *mut u8,
                data.len(),
            );

            // Descriptor 길이 업데이트
            self.tx_vring.set_desc(
                desc_idx,
                buf_addr,
                (core::mem::size_of::<RpmsgHdr>() + data.len()) as u32,
                0,
            );
        }

        // Available ring에 추가
        self.tx_vring.add_available(desc_idx);

        // Mailbox로 A15에 통지 (vring1 = TX)
        cortex_m::asm::dsb();
        self.mailbox.send_to_a15(1) // vring index = 1 (TX)
    }

    /// Mailbox IRQ 클리어
    pub fn clear_irq(&self) {
        self.mailbox.clear_irq();
    }

    /// 수신 대기 중인 메시지 확인
    pub fn has_pending(&self) -> bool {
        self.mailbox.has_pending_message()
    }
}
