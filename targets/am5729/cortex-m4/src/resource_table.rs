//! ─────────────────────────────────────────────────────────────
//! remoteproc Resource Table (M4 펌웨어 헤더)
//! ─────────────────────────────────────────────────────────────
//!
//! Linux remoteproc 드라이버가 M4 펌웨어를 로드할 때
//! 이 리소스 테이블을 파싱하여 다음을 설정합니다:
//!
//!   1. RSC_CARVEOUT — M4용 메모리 영역 할당
//!   2. RSC_DEVMEM   — Device-to-PA 메모리 매핑
//!   3. RSC_VDEV     — VirtIO 디바이스 (rpmsg용 vring 2개)
//!   4. RSC_TRACE    — 디버그 트레이스 버퍼 위치
//!
//! ELF 섹션 `.resource_table`에 배치되어
//! remoteproc이 펌웨어 로드 시 자동으로 찾습니다.
//!
//! 참고:
//!   - Linux: include/linux/remoteproc.h
//!   - TI: ti-ipc-rtos/packages/ti/ipc/remoteproc/rsc_types.h
//!   - TRM SPRUHZ6 — Chapter 6: IPC

/// Resource type 열거
pub mod rsc_type {
    pub const RSC_CARVEOUT: u32 = 0;
    pub const RSC_DEVMEM: u32 = 1;
    pub const RSC_TRACE: u32 = 2;
    pub const RSC_VDEV: u32 = 3;
}

/// VirtIO 디바이스 ID
pub const VIRTIO_ID_RPMSG: u32 = 7;

/// VirtIO 피처 플래그
pub const VIRTIO_RPMSG_F_NS: u32 = 0; // Name Service 지원 bit position

/// Resource Table 헤더
#[repr(C)]
pub struct ResourceTable {
    pub ver: u32,
    pub num: u32,
    pub reserved: [u32; 2],
    pub offset: [u32; NUM_RESOURCES],
}

/// 리소스 수
pub const NUM_RESOURCES: usize = 4;

// ── 각 리소스 엔트리 구조체 ──

/// Carve-out 메모리 리소스
#[repr(C)]
pub struct RscCarveout {
    pub rsc_type: u32,
    pub da: u32,         // Device Address (M4 관점)
    pub pa: u32,         // Physical Address (A15 관점)
    pub len: u32,
    pub flags: u32,
    pub reserved: u32,
    pub name: [u8; 32],
}

/// Device Memory 매핑 리소스
#[repr(C)]
pub struct RscDevmem {
    pub rsc_type: u32,
    pub da: u32,
    pub pa: u32,
    pub len: u32,
    pub flags: u32,
    pub reserved: u32,
    pub name: [u8; 32],
}

/// Trace 버퍼 리소스
#[repr(C)]
pub struct RscTrace {
    pub rsc_type: u32,
    pub da: u32,
    pub len: u32,
    pub reserved: u32,
    pub name: [u8; 32],
}

/// VirtIO 디바이스 리소스 (rpmsg용)
#[repr(C)]
pub struct RscVdev {
    pub rsc_type: u32,
    pub id: u32,           // VIRTIO_ID_RPMSG = 7
    pub notifyid: u32,
    pub dfeatures: u32,    // 디바이스 피처
    pub gfeatures: u32,    // 게스트 피처
    pub config_len: u32,
    pub status: u8,
    pub num_of_vrings: u8,
    pub reserved: [u8; 2],
    // 이후 FwRscVdevVring × num_of_vrings 가 따라옴
}

/// VirtIO Vring 설정 (리소스 테이블 내)
#[repr(C)]
pub struct FwRscVdevVring {
    pub da: u32,           // Vring Device Address (0 = 자동 할당)
    pub align: u32,        // 정렬 (4096)
    pub num: u32,          // Descriptor 수 (256)
    pub notifyid: u32,     // 알림 ID
    pub reserved: u32,
}

// ── 전체 리소스 테이블 레이아웃 ──

/// AM5729 IPU1 전체 리소스 테이블
/// remoteproc이 이 구조체를 파싱합니다
#[repr(C)]
pub struct Am5729IpuResourceTable {
    pub header: ResourceTable,
    // Resource 0: IPU RAM carve-out
    pub ipu_mem: RscCarveout,
    // Resource 1: IPC 공유 메모리 carve-out
    pub ipc_mem: RscCarveout,
    // Resource 2: VirtIO rpmsg 디바이스
    pub rpmsg_vdev: RscVdev,
    pub rpmsg_vring0: FwRscVdevVring, // RX (A15 → M4)
    pub rpmsg_vring1: FwRscVdevVring, // TX (M4 → A15)
    // Resource 3: Trace 버퍼
    pub trace: RscTrace,
}

// ── 메모리 설정 상수 ──

/// IPU1 내부 RAM (AMMU 매핑 후)
const IPU_MEM_DA: u32 = 0x0000_0000;
const IPU_MEM_LEN: u32 = 64 * 1024; // 64KB

/// IPC 공유 메모리 (rpmsg 버퍼)
const IPC_MEM_DA: u32 = 0x9F00_0000;
const IPC_MEM_PA: u32 = 0x9F00_0000;
const IPC_MEM_LEN: u32 = 1024 * 1024; // 1MB

/// 트레이스 버퍼 (디버그 로그)
const TRACE_BUF_DA: u32 = 0x0000_E000;
const TRACE_BUF_LEN: u32 = 0x2000; // 8KB

/// Vring 설정
const VRING_ALIGN: u32 = 4096;
const VRING_NUM: u32 = 256;

// ── 트레이스 버퍼 (실제 메모리) ──

/// defmt/로그 출력이 이 버퍼에 기록됩니다
/// remoteproc이 이를 읽어서 /sys/kernel/debug/remoteproc/remoteprocN/trace0 에 노출
#[link_section = ".trace_buf"]
#[no_mangle]
static mut TRACE_BUF: [u8; TRACE_BUF_LEN as usize] = [0u8; TRACE_BUF_LEN as usize];

// ── 리소스 테이블 인스턴스 (ELF .resource_table 섹션) ──

/// 이름 문자열을 [u8; 32] 배열로 변환하는 헬퍼
const fn name_bytes(s: &[u8]) -> [u8; 32] {
    let mut buf = [0u8; 32];
    let len = if s.len() < 32 { s.len() } else { 31 };
    let mut i = 0;
    while i < len {
        buf[i] = s[i];
        i += 1;
    }
    buf
}

/// 각 리소스의 offset 계산 헬퍼
/// header는 4*u32 + NUM_RESOURCES*u32 크기
const HEADER_SIZE: u32 = 4 * 4 + (NUM_RESOURCES as u32) * 4;
const CARVEOUT_SIZE: u32 = 6 * 4 + 32;           // RscCarveout = 56 bytes
const VDEV_SIZE: u32 = 7 * 4 + 1 + 1 + 2;       // RscVdev = 30 bytes
const VRING_CFG_SIZE: u32 = 5 * 4;                // FwRscVdevVring = 20 bytes
const TRACE_SIZE: u32 = 4 * 4 + 32;               // RscTrace = 48 bytes

const OFFSET_0: u32 = HEADER_SIZE;                                    // ipu_mem
const OFFSET_1: u32 = OFFSET_0 + CARVEOUT_SIZE;                       // ipc_mem
const OFFSET_2: u32 = OFFSET_1 + CARVEOUT_SIZE;                       // rpmsg_vdev
const OFFSET_3: u32 = OFFSET_2 + VDEV_SIZE + 2 * VRING_CFG_SIZE;     // trace

#[link_section = ".resource_table"]
#[no_mangle]
#[used]
pub static RESOURCE_TABLE: Am5729IpuResourceTable = Am5729IpuResourceTable {
    header: ResourceTable {
        ver: 1,
        num: NUM_RESOURCES as u32,
        reserved: [0, 0],
        offset: [OFFSET_0, OFFSET_1, OFFSET_2, OFFSET_3],
    },

    // Resource 0: IPU 내부 RAM
    ipu_mem: RscCarveout {
        rsc_type: rsc_type::RSC_CARVEOUT,
        da: IPU_MEM_DA,
        pa: 0x5502_0000, // AM5729 IPU1 물리 주소
        len: IPU_MEM_LEN,
        flags: 0,
        reserved: 0,
        name: name_bytes(b"IPU1_MEM"),
    },

    // Resource 1: IPC 공유 메모리
    ipc_mem: RscCarveout {
        rsc_type: rsc_type::RSC_CARVEOUT,
        da: IPC_MEM_DA,
        pa: IPC_MEM_PA,
        len: IPC_MEM_LEN,
        flags: 0,
        reserved: 0,
        name: name_bytes(b"IPC_MEM"),
    },

    // Resource 2: rpmsg VirtIO 디바이스
    rpmsg_vdev: RscVdev {
        rsc_type: rsc_type::RSC_VDEV,
        id: VIRTIO_ID_RPMSG,
        notifyid: 0,
        dfeatures: 1 << VIRTIO_RPMSG_F_NS, // Name Service 지원
        gfeatures: 0,
        config_len: 0,
        status: 0,
        num_of_vrings: 2,
        reserved: [0, 0],
    },

    // RX vring (A15 → M4)
    rpmsg_vring0: FwRscVdevVring {
        da: 0,             // 0 = Linux가 자동 할당
        align: VRING_ALIGN,
        num: VRING_NUM,
        notifyid: 0,       // Mailbox sub-mailbox ID
        reserved: 0,
    },

    // TX vring (M4 → A15)
    rpmsg_vring1: FwRscVdevVring {
        da: 0,
        align: VRING_ALIGN,
        num: VRING_NUM,
        notifyid: 1,
        reserved: 0,
    },

    // Resource 3: 디버그 트레이스 버퍼
    trace: RscTrace {
        rsc_type: rsc_type::RSC_TRACE,
        da: TRACE_BUF_DA,
        len: TRACE_BUF_LEN,
        reserved: 0,
        name: name_bytes(b"ipu1_trace"),
    },
};
