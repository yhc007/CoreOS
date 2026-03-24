//! ─────────────────────────────────────────────────────────────
//! VirtIO Vring 구현 (rpmsg 전송 계층)
//! ─────────────────────────────────────────────────────────────
//!
//! rpmsg는 VirtIO 위에 구축됩니다.
//! M4 펌웨어는 2개의 vring을 사용:
//!   - vring0 (RX): A15 → M4 수신 버퍼
//!   - vring1 (TX): M4 → A15 송신 버퍼
//!
//! 각 vring 구조:
//!   ┌──────────────────────────┐
//!   │  Descriptor Table        │  — 512 entries (16 bytes each)
//!   │  [addr | len | flags | next] │
//!   ├──────────────────────────┤
//!   │  Available Ring          │  — 호스트가 읽을 desc index 목록
//!   │  [flags | idx | ring[N]] │
//!   ├──────────────────────────┤
//!   │  Used Ring               │  — 디바이스가 처리 완료한 desc
//!   │  [flags | idx | ring[N]] │
//!   └──────────────────────────┘
//!
//! 참고: VirtIO 1.0 Specification, Linux virtio_ring.h

/// Vring 설정 상수 (remoteproc 관례)
pub const VRING_SIZE: usize = 256;           // descriptor 수
pub const VRING_ALIGN: usize = 4096;         // 페이지 정렬

/// rpmsg 버퍼 크기 (Linux rpmsg 기본값)
pub const RPMSG_BUF_SIZE: usize = 512;

/// Vring Descriptor (16 bytes, packed)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct VringDesc {
    /// 버퍼 물리 주소
    pub addr: u64,
    /// 버퍼 길이
    pub len: u32,
    /// 플래그 (NEXT, WRITE, INDIRECT)
    pub flags: u16,
    /// 다음 descriptor 인덱스 (chaining)
    pub next: u16,
}

/// Descriptor 플래그
pub mod desc_flags {
    /// 다음 descriptor로 체인
    pub const VRING_DESC_F_NEXT: u16 = 1;
    /// 디바이스 쓰기용 (호스트 → 디바이스)
    pub const VRING_DESC_F_WRITE: u16 = 2;
    /// Indirect descriptor
    pub const VRING_DESC_F_INDIRECT: u16 = 4;
}

/// Available Ring (호스트가 게스트에 제공하는 버퍼 목록)
#[repr(C)]
pub struct VringAvail {
    pub flags: u16,
    pub idx: u16,
    pub ring: [u16; VRING_SIZE],
    /// used_event (VIRTIO_F_EVENT_IDX)
    pub used_event: u16,
}

/// Used Ring Element
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct VringUsedElem {
    /// 사용된 descriptor chain의 head 인덱스
    pub id: u32,
    /// 실제 쓰여진 바이트 수
    pub len: u32,
}

/// Used Ring (게스트가 호스트에 반환하는 완료 버퍼)
#[repr(C)]
pub struct VringUsed {
    pub flags: u16,
    pub idx: u16,
    pub ring: [VringUsedElem; VRING_SIZE],
    /// avail_event (VIRTIO_F_EVENT_IDX)
    pub avail_event: u16,
}

/// Vring 인스턴스 (M4 측에서 관리)
pub struct Vring {
    /// Descriptor table 시작 주소
    pub desc_addr: u32,
    /// Available ring 시작 주소
    pub avail_addr: u32,
    /// Used ring 시작 주소
    pub used_addr: u32,
    /// Vring 크기 (descriptor 수)
    pub num: u16,
    /// 마지막으로 확인한 avail idx (RX용)
    pub last_avail_idx: u16,
    /// 마지막으로 확인한 used idx (TX용)
    pub last_used_idx: u16,
}

impl Vring {
    /// Resource table에서 받은 주소로 vring 초기화
    pub fn new(base_addr: u32, num: u16) -> Self {
        let desc_size = (num as u32) * 16; // sizeof(VringDesc) = 16
        let avail_size = 6 + (num as u32) * 2; // flags + idx + ring[N] + used_event
        let avail_addr = base_addr + desc_size;
        // used ring은 VRING_ALIGN 경계에 정렬
        let used_addr = align_up(avail_addr + avail_size, VRING_ALIGN as u32);

        Self {
            desc_addr: base_addr,
            avail_addr,
            used_addr,
            num,
            last_avail_idx: 0,
            last_used_idx: 0,
        }
    }

    /// Available ring에 새 버퍼가 있는지 확인
    pub fn has_available(&self) -> bool {
        unsafe {
            let avail_idx = core::ptr::read_volatile(
                (self.avail_addr + 2) as *const u16 // avail->idx
            );
            avail_idx != self.last_avail_idx
        }
    }

    /// Available ring에서 다음 descriptor 인덱스 가져오기
    pub fn get_available(&mut self) -> Option<u16> {
        if !self.has_available() {
            return None;
        }
        unsafe {
            let ring_idx = (self.last_avail_idx % self.num) as u32;
            let desc_idx = core::ptr::read_volatile(
                (self.avail_addr + 4 + ring_idx * 2) as *const u16 // avail->ring[idx]
            );
            self.last_avail_idx = self.last_avail_idx.wrapping_add(1);
            Some(desc_idx)
        }
    }

    /// Descriptor에서 버퍼 주소와 길이 읽기
    pub fn get_desc(&self, idx: u16) -> (u32, u32, u16) {
        unsafe {
            let desc_ptr = (self.desc_addr + (idx as u32) * 16) as *const u8;
            let addr = core::ptr::read_volatile(desc_ptr as *const u64) as u32;
            let len = core::ptr::read_volatile(desc_ptr.add(8) as *const u32);
            let flags = core::ptr::read_volatile(desc_ptr.add(12) as *const u16);
            (addr, len, flags)
        }
    }

    /// Used ring에 완료된 버퍼 추가
    pub fn add_used(&mut self, desc_idx: u16, len: u32) {
        unsafe {
            let used_idx_ptr = (self.used_addr + 2) as *mut u16;
            let current_used = core::ptr::read_volatile(used_idx_ptr);
            let ring_idx = (current_used % self.num) as u32;

            // used->ring[idx].id = desc_idx
            let elem_addr = self.used_addr + 4 + ring_idx * 8;
            core::ptr::write_volatile(elem_addr as *mut u32, desc_idx as u32);
            // used->ring[idx].len = len
            core::ptr::write_volatile((elem_addr + 4) as *mut u32, len);

            // Memory barrier 후 used->idx 증가
            cortex_m::asm::dsb();
            core::ptr::write_volatile(used_idx_ptr, current_used.wrapping_add(1));
        }
    }

    /// Descriptor에 버퍼 설정 (TX용)
    pub fn set_desc(&self, idx: u16, addr: u32, len: u32, flags: u16) {
        unsafe {
            let desc_ptr = (self.desc_addr + (idx as u32) * 16) as *mut u8;
            core::ptr::write_volatile(desc_ptr as *mut u64, addr as u64);
            core::ptr::write_volatile(desc_ptr.add(8) as *mut u32, len);
            core::ptr::write_volatile(desc_ptr.add(12) as *mut u16, flags);
        }
    }

    /// Available ring에 descriptor 추가 (TX용)
    pub fn add_available(&mut self, desc_idx: u16) {
        unsafe {
            let avail_idx_ptr = (self.avail_addr + 2) as *mut u16;
            let current_avail = core::ptr::read_volatile(avail_idx_ptr);
            let ring_idx = (current_avail % self.num) as u32;

            // avail->ring[idx] = desc_idx
            core::ptr::write_volatile(
                (self.avail_addr + 4 + ring_idx * 2) as *mut u16,
                desc_idx,
            );

            cortex_m::asm::dsb();
            core::ptr::write_volatile(avail_idx_ptr, current_avail.wrapping_add(1));
        }
    }
}

/// 주소를 alignment 경계로 올림
fn align_up(addr: u32, align: u32) -> u32 {
    (addr + align - 1) & !(align - 1)
}
