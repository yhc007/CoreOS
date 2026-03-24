//! ─────────────────────────────────────────────────────────────
//! AM5729 IPU1 Unicache AMMU (Attribute Memory Management Unit)
//! ─────────────────────────────────────────────────────────────
//!
//! AM5729 IPU의 Cortex-M4는 물리 버스에 직접 접근하지 않습니다.
//! 모든 메모리 접근은 Unicache → AMMU를 거쳐 L3 인터커넥트로 나갑니다.
//!
//! AMMU가 없으면:
//!   - L4_PER(0x4800_0000) 페리페럴 접근 불가 → GPIO, UART, Timer 등 전부 사용 불가
//!   - OCMC RAM 접근 불가
//!   - IPC 공유 메모리 접근 불가
//!
//! AMMU 페이지 구조 (IPU1 Unicache):
//!   ┌─────────────────────────────────────────────────────┐
//!   │  Large Pages  (4개)  — 512MB 단위 매핑              │
//!   │  Medium Pages (2개)  — 256KB 단위 매핑              │
//!   │  Small Pages  (10개) — 16KB 단위 매핑               │
//!   │  Line Pages   (16개) — 32B 단위 (캐시 라인)         │
//!   └─────────────────────────────────────────────────────┘
//!
//! 각 페이지는 3개의 레지스터로 설정:
//!   1. LOGICAL_ADDR — M4가 보는 가상 주소
//!   2. PHYSICAL_ADDR — L3 인터커넥트의 실제 물리 주소
//!   3. POLICY — 캐시 정책 + 활성화 비트
//!
//! 주의: AMMU 설정은 반드시 다른 모든 페리페럴 초기화보다 **먼저** 수행해야 함!
//!       cortex-m-rt의 Reset_Handler에서 main() 진입 전에 호출하는 것이 이상적.
//!
//! 참고:
//!   - TRM SPRUHZ6 — Chapter 7: Unicache (IPU)
//!   - TRM SPRUHZ6 — Section 7.4: AMMU
//!   - TRM SPRUHZ6 — Table 7-14 ~ 7-17: AMMU Registers
//!   - TI IPC SDK: packages/ti/ipc/family/am57xx/AmmuCfg.c

// ═══════════════════════════════════════════════════════════
// AMMU 레지스터 베이스 주소
// ═══════════════════════════════════════════════════════════
//
// IPU1 Unicache AMMU는 IPU 내부 주소 공간에 매핑됩니다.
// 이 주소 자체는 AMMU 변환 **이전**에 접근 가능합니다 (내부 레지스터이므로).
//
// IPU1 Unicache 베이스: 0x5508_0000 (물리)
// BUT — M4 관점에서는 AMMU 자체 레지스터가 내부적으로 매핑된 주소를 사용합니다.
//
// TRM Table 7-14: IPU Unicache MMU 레지스터
// IPU1 내부 관점에서의 AMMU 레지스터 베이스 주소:
//   0x40000000 + 오프셋 (Unicache Config space)
//
// 실제로 TI IPC SDK와 SYS/BIOS에서 사용하는 주소:

/// Unicache AMMU 레지스터 베이스 (IPU 내부 주소 공간)
///
/// IPU Unicache 설정 공간:
///   IPU1: 물리 0x5508_0000, IPU 내부 0x0008_0000 (초기 AMMU 매핑)
///
/// 주의: 리셋 직후 IPU의 기본 AMMU 설정에서는
///       0x00000000 → 0x55020000 (64KB, IPU RAM)
///       0x00080000 → 0x55080000 (Unicache Config)
///       이 기본 매핑 덕분에 AMMU 레지스터 자체에는 접근 가능합니다.
const AMMU_BASE: u32 = 0x0008_0000;

// ═══════════════════════════════════════════════════════════
// 레지스터 오프셋
// ═══════════════════════════════════════════════════════════

/// Large Page 레지스터 (4 pages × 512MB)
pub mod large_page {
    /// 각 Large Page의 오프셋 (page 0~3)
    pub const LOGICAL_ADDR: [u32; 4] = [0x260, 0x268, 0x270, 0x278];
    pub const PHYSICAL_ADDR: [u32; 4] = [0x264, 0x26C, 0x274, 0x27C];
    pub const POLICY: [u32; 4] = [0x2A0, 0x2A4, 0x2A8, 0x2AC];
}

/// Medium Page 레지스터 (2 pages × 256KB)
pub mod medium_page {
    pub const LOGICAL_ADDR: [u32; 2] = [0x2C0, 0x2C8];
    pub const PHYSICAL_ADDR: [u32; 2] = [0x2C4, 0x2CC];
    pub const POLICY: [u32; 2] = [0x2E0, 0x2E4];
}

/// Small Page 레지스터 (10 pages × 16KB)
pub mod small_page {
    pub const LOGICAL_ADDR: [u32; 10] = [
        0x300, 0x308, 0x310, 0x318, 0x320,
        0x328, 0x330, 0x338, 0x340, 0x348,
    ];
    pub const PHYSICAL_ADDR: [u32; 10] = [
        0x304, 0x30C, 0x314, 0x31C, 0x324,
        0x32C, 0x334, 0x33C, 0x344, 0x34C,
    ];
    pub const POLICY: [u32; 10] = [
        0x360, 0x364, 0x368, 0x36C, 0x370,
        0x374, 0x378, 0x37C, 0x380, 0x384,
    ];
}

// ═══════════════════════════════════════════════════════════
// 정책 (Policy) 비트 필드
// ═══════════════════════════════════════════════════════════
//
// TRM Section 7.4.3 — AMMU Page Policy Register
//
//   Bit 0:    Enable (1 = 페이지 활성화)
//   Bit 1:    Size override (Large pages에서 사용)
//   Bit 4:    Volatile (1 = 캐시하지 않음, 페리페럴에 필수!)
//   Bit 5:    Cacheable (1 = L1 캐시 사용)
//   Bit 6:    Posted (1 = 쓰기 버퍼 사용)
//   Bit 7:    Allocate (1 = 캐시 미스 시 라인 할당)
//   Bit 8:    Write-back (1 = Write-back, 0 = Write-through)
//   Bit 16:   Prefetch (1 = 프리페치 활성화)
//   Bit 17:   Exclusive (1 = 이 페이지 전용 캐시)

pub mod policy {
    /// 페이지 활성화
    pub const ENABLE: u32 = 1 << 0;

    // ── 캐시 정책 조합 ──

    /// 캐시 불가 (Volatile) — 페리페럴 레지스터에 사용
    /// 모든 읽기/쓰기가 즉시 물리 버스로 전달됨
    pub const NON_CACHEABLE: u32 = (1 << 4) | ENABLE;

    /// 캐시 가능 + Write-through — 코드/읽기 전용 데이터에 사용
    pub const CACHEABLE_WT: u32 = (1 << 5) | (1 << 7) | ENABLE;

    /// 캐시 가능 + Write-back — RAM 데이터에 사용 (성능 최적)
    pub const CACHEABLE_WB: u32 = (1 << 5) | (1 << 7) | (1 << 8) | ENABLE;

    /// 캐시 가능 + Write-back + Posted — 고성능 메모리 접근
    pub const CACHEABLE_WB_POSTED: u32 = (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | ENABLE;

    /// 캐시 불가 + Posted — 공유 메모리(IPC)에 적합
    /// 캐시 코히런시 문제 없이 빠른 쓰기
    pub const NON_CACHEABLE_POSTED: u32 = (1 << 4) | (1 << 6) | ENABLE;

    /// Large Page 크기 (512MB = 0x2000_0000)
    /// Large page에서는 size 비트가 사실상 주소 정렬로 결정됨
    pub const LARGE_512MB: u32 = 0;

    /// Prefetch 활성화 (코드 영역에 유용)
    pub const PREFETCH: u32 = 1 << 16;
}

// ═══════════════════════════════════════════════════════════
// AMMU 설정 구조체
// ═══════════════════════════════════════════════════════════

/// 페이지 매핑 설정 항목
#[derive(Clone, Copy)]
pub struct PageMapping {
    pub logical: u32,
    pub physical: u32,
    pub policy: u32,
}

/// AMMU 드라이버
pub struct Ammu;

impl Ammu {
    /// 레지스터 쓰기 (AMMU 내부 주소 공간)
    #[inline(always)]
    unsafe fn write(offset: u32, val: u32) {
        core::ptr::write_volatile((AMMU_BASE + offset) as *mut u32, val);
    }

    /// 레지스터 읽기
    #[inline(always)]
    unsafe fn read(offset: u32) -> u32 {
        core::ptr::read_volatile((AMMU_BASE + offset) as *const u32)
    }

    /// Large Page 설정 (512MB 단위, 4개)
    pub fn set_large_page(index: usize, mapping: &PageMapping) {
        assert!(index < 4, "Large page index must be 0..3");
        unsafe {
            Self::write(large_page::LOGICAL_ADDR[index], mapping.logical);
            Self::write(large_page::PHYSICAL_ADDR[index], mapping.physical);
            Self::write(large_page::POLICY[index], mapping.policy);
        }
    }

    /// Medium Page 설정 (256KB 단위, 2개)
    pub fn set_medium_page(index: usize, mapping: &PageMapping) {
        assert!(index < 2, "Medium page index must be 0..1");
        unsafe {
            Self::write(medium_page::LOGICAL_ADDR[index], mapping.logical);
            Self::write(medium_page::PHYSICAL_ADDR[index], mapping.physical);
            Self::write(medium_page::POLICY[index], mapping.policy);
        }
    }

    /// Small Page 설정 (16KB 단위, 10개)
    pub fn set_small_page(index: usize, mapping: &PageMapping) {
        assert!(index < 10, "Small page index must be 0..9");
        unsafe {
            Self::write(small_page::LOGICAL_ADDR[index], mapping.logical);
            Self::write(small_page::PHYSICAL_ADDR[index], mapping.physical);
            Self::write(small_page::POLICY[index], mapping.policy);
        }
    }

    /// Large Page 현재 설정 읽기 (디버그용)
    pub fn get_large_page(index: usize) -> PageMapping {
        assert!(index < 4);
        unsafe {
            PageMapping {
                logical: Self::read(large_page::LOGICAL_ADDR[index]),
                physical: Self::read(large_page::PHYSICAL_ADDR[index]),
                policy: Self::read(large_page::POLICY[index]),
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════
// IPU1 AMMU 초기화 — 이것이 핵심!
// ═══════════════════════════════════════════════════════════
//
// AM5729 IPU1의 AMMU를 로봇 제어 펌웨어에 맞게 설정합니다.
//
// 리셋 후 기본 매핑 (TI-RTOS/remoteproc 설정):
//   Large Page 0: 0x40000000 → 0x40000000 (L3_MAIN, OCMC)     — 기본 활성화
//   Large Page 1: 0x80000000 → 0x80000000 (DDR)                — 기본 활성화
//   Large Page 2: 비활성화
//   Large Page 3: 비활성화
//
// 우리가 필요한 매핑:
//
// ┌──────────────────────────────────────────────────────────────────┐
// │ Type    │ IPU 논리 주소      → 물리 주소           │ 용도         │
// ├─────────┼──────────────────────────────────────────┼──────────────┤
// │ Large 0 │ 0x4000_0000 → 0x4000_0000 (512MB)       │ L3_MAIN      │
// │         │   └ 0x4030_0000: OCMC RAM1 (512KB)      │ 코드/데이터  │
// │         │   └ 0x4050_0000: OCMC RAM3              │ 확장 버퍼    │
// ├─────────┼──────────────────────────────────────────┼──────────────┤
// │ Large 1 │ 0x4800_0000 → 0x4800_0000 (512MB)       │ L4_PER       │
// │         │   └ 0x4805_xxxx: GPIO2~8                │ LED, E-Stop  │
// │         │   └ 0x4806_xxxx: UART, I2C              │ 센서 통신    │
// │         │   └ 0x4803_xxxx: Timer                  │ 제어 타이머  │
// │         │   └ 0x481C_xxxx: DCAN1/2                │ CAN Bus      │
// │         │   └ 0x4843_xxxx: ePWM, eQEP             │ 모터 제어    │
// │         │   └ 0x4883_xxxx: Mailbox5/6             │ rpmsg IPC    │
// ├─────────┼──────────────────────────────────────────┼──────────────┤
// │ Large 2 │ 0x8000_0000 → 0x8000_0000 (512MB)       │ DDR (일부)   │
// │         │   └ DMA 버퍼, 대용량 데이터              │              │
// ├─────────┼──────────────────────────────────────────┼──────────────┤
// │ Large 3 │ 0x9F00_0000 → 0x9F00_0000 (512MB 중 1MB)│ IPC Shared   │
// │         │   └ rpmsg VirtIO vring 버퍼              │ A15↔M4       │
// │         │   주의: 0x8000_0000~0x9FFF_FFFF는 Large 2와 겹침       │
// │         │   → Small Page로 별도 매핑 또는 Large 2에서 커버        │
// ├─────────┼──────────────────────────────────────────┼──────────────┤
// │ Small 0 │ 0x4AE1_0000 → 0x4AE1_0000 (16KB)        │ GPIO1 (WKUP)│
// │         │   L4_WKUP 영역은 Large 1(L4_PER)에       │              │
// │         │   포함 안 됨 → 별도 매핑 필요             │              │
// └──────────────────────────────────────────────────────────────────┘

/// AM5729 IPU1 AMMU 전체 초기화
///
/// **반드시 다른 모든 페리페럴 접근 전에 호출해야 합니다!**
///
/// 이 함수가 실행된 후에만:
///   - GPIO, UART, Timer, DCAN, ePWM, eQEP 등의 L4_PER 페리페럴 접근 가능
///   - OCMC RAM1/3 접근 가능
///   - rpmsg IPC 공유 메모리 접근 가능
///   - DDR 메모리 접근 가능
pub fn init_ammu() {
    // ──────────────────────────────────────────────
    // Large Page 0: L3_MAIN (OCMC RAM 포함)
    // 논리 0x4000_0000 → 물리 0x4000_0000 (1:1)
    // OCMC RAM1 @ 0x4030_0000 (512KB)
    // 캐시 가능 (Write-back) — 코드 + 데이터 영역
    // ──────────────────────────────────────────────
    Ammu::set_large_page(0, &PageMapping {
        logical:  0x4000_0000,
        physical: 0x4000_0000,
        policy:   policy::CACHEABLE_WB_POSTED | policy::PREFETCH,
    });

    // ──────────────────────────────────────────────
    // Large Page 1: L4_PER + L4_CFG (페리페럴 전체)
    // 논리 0x4800_0000 → 물리 0x4800_0000 (1:1)
    //
    // 이 매핑으로 접근 가능해지는 페리페럴:
    //   0x4803_xxxx — Timer2/3/4
    //   0x4805_xxxx — GPIO2~8
    //   0x4806_xxxx — I2C1~3, UART1~3
    //   0x481C_xxxx — DCAN1/2
    //   0x4843_xxxx — PWMSS (ePWM, eQEP, eCAP)
    //   0x4883_xxxx — Mailbox5/6 (rpmsg IPC)
    //
    // 캐시 불가 (Non-Cacheable) — 페리페럴은 반드시!
    // 캐시하면 레지스터 읽기/쓰기가 지연되어 실시간 제어 불가
    // ──────────────────────────────────────────────
    Ammu::set_large_page(1, &PageMapping {
        logical:  0x4800_0000,
        physical: 0x4800_0000,
        policy:   policy::NON_CACHEABLE,
    });

    // ──────────────────────────────────────────────
    // Large Page 2: DDR 메모리 영역
    // 논리 0x8000_0000 → 물리 0x8000_0000 (1:1)
    //
    // DDR3/4 메모리 접근 (DMA 버퍼, 대용량 데이터)
    // IPC 공유 메모리(0x9F00_0000)도 이 범위에 포함됨:
    //   0x8000_0000 ~ 0x9FFF_FFFF = 512MB 범위
    //
    // 캐시 불가 + Posted — IPC 공유 메모리는 캐시 코히런시 문제 방지
    // (A15와 M4가 동시에 접근하므로 캐시하면 안 됨)
    // ──────────────────────────────────────────────
    Ammu::set_large_page(2, &PageMapping {
        logical:  0x8000_0000,
        physical: 0x8000_0000,
        policy:   policy::NON_CACHEABLE_POSTED,
    });

    // ──────────────────────────────────────────────
    // Large Page 3: L4_WKUP + L4_CFG 상위 영역
    // 논리 0x4A00_0000 → 물리 0x4A00_0000 (1:1)
    //
    // L4_WKUP 영역 페리페럴:
    //   0x4AE1_0000 — GPIO1 (Wakeup 도메인)
    //   0x4AE0_xxxx — PRM (Power/Reset Manager)
    //   0x4A00_xxxx — CM_CORE 등 시스템 설정 레지스터
    //
    // 캐시 불가 — 페리페럴 영역
    // ──────────────────────────────────────────────
    Ammu::set_large_page(3, &PageMapping {
        logical:  0x4A00_0000,
        physical: 0x4A00_0000,
        policy:   policy::NON_CACHEABLE,
    });

    // ──────────────────────────────────────────────
    // Medium Page 0: (예비 — 필요 시 256KB 단위 매핑)
    // 예: 특정 OCMC 영역을 캐시 정책 다르게 하고 싶을 때
    // ──────────────────────────────────────────────
    // 현재는 미사용 — Large Page로 충분

    // ──────────────────────────────────────────────
    // Small Page 0: IPU1 내부 타이머 (CTM)
    // 논리 0x0004_0000 → 물리 0x5504_0000
    //
    // IPU Counter Timer Module — 정밀 타이밍 측정에 사용
    // (SysTick 외에 별도의 하드웨어 카운터)
    // ──────────────────────────────────────────────
    Ammu::set_small_page(0, &PageMapping {
        logical:  0x0004_0000,
        physical: 0x5504_0000,
        policy:   policy::NON_CACHEABLE,
    });

    // ──────────────────────────────────────────────
    // Small Page 1: IPU1 INTC (Interrupt Controller)
    // 논리 0x000E_0000 → 물리 0x550E_0000
    //
    // IPU 전용 인터럽트 컨트롤러 레지스터
    // RTIC dispatchers가 사용하는 인터럽트 관리에 필요
    // ──────────────────────────────────────────────
    Ammu::set_small_page(1, &PageMapping {
        logical:  0x000E_0000,
        physical: 0x550E_0000,
        policy:   policy::NON_CACHEABLE,
    });

    // ──────────────────────────────────────────────
    // 메모리 배리어: AMMU 설정이 완전히 적용되도록
    // DSB + ISB로 파이프라인 및 캐시 동기화
    // ──────────────────────────────────────────────
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

// ═══════════════════════════════════════════════════════════
// AMMU 디버그 출력
// ═══════════════════════════════════════════════════════════

/// 현재 AMMU 설정을 defmt로 출력 (디버그용)
#[cfg(feature = "debug-ammu")]
pub fn dump_ammu_config() {
    defmt::info!("=== AMMU Configuration ===");
    for i in 0..4 {
        let p = Ammu::get_large_page(i);
        if p.policy & policy::ENABLE != 0 {
            defmt::info!(
                "  Large[{}]: 0x{:08X} → 0x{:08X} policy=0x{:08X}",
                i, p.logical, p.physical, p.policy
            );
        }
    }
}

// ═══════════════════════════════════════════════════════════
// AMMU 매핑 검증 (테스트/디버그용)
// ═══════════════════════════════════════════════════════════

/// AMMU 매핑이 올바른지 검증
///
/// 각 페리페럴의 Module ID 레지스터를 읽어서
/// 예상값과 비교합니다.
///
/// 반환: 통과한 페리페럴 수
pub fn verify_ammu_mappings() -> u32 {
    let mut passed = 0u32;

    unsafe {
        // GPIO2 Module ID (offset 0x000) = Revision register
        // 정상이면 0이 아닌 값
        let gpio2_rev = core::ptr::read_volatile(0x4805_5000u32 as *const u32);
        if gpio2_rev != 0 && gpio2_rev != 0xFFFF_FFFF {
            passed += 1;
        }

        // UART3 Module ID
        let uart3_rev = core::ptr::read_volatile(0x4802_0050u32 as *const u32);
        if uart3_rev != 0 && uart3_rev != 0xFFFF_FFFF {
            passed += 1;
        }

        // Timer2 Module ID
        let timer2_rev = core::ptr::read_volatile(0x4803_2000u32 as *const u32);
        if timer2_rev != 0 && timer2_rev != 0xFFFF_FFFF {
            passed += 1;
        }

        // Mailbox5 Module ID (offset 0x000 = REVISION)
        let mbox5_rev = core::ptr::read_volatile(0x4883_A000u32 as *const u32);
        if mbox5_rev != 0 && mbox5_rev != 0xFFFF_FFFF {
            passed += 1;
        }

        // DCAN1 Module ID
        let dcan1_rev = core::ptr::read_volatile(0x481C_C020u32 as *const u32);
        if dcan1_rev != 0 && dcan1_rev != 0xFFFF_FFFF {
            passed += 1;
        }
    }

    passed
}
