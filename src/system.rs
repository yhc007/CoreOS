//! ─────────────────────────────────────────────────────────────
//! 시스템 공유 상태 — 태스크 간 공유되는 전역 상태
//! ─────────────────────────────────────────────────────────────

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_time::Instant;
use rtos_esp32s3::drivers::protocol::SystemStatusReport;

/// E-Stop 래치 — 한 번 걸리면 재부팅 전까지 해제되지 않음
static ESTOP: AtomicBool = AtomicBool::new(false);

/// E-Stop 발동
pub fn trigger_estop() {
    ESTOP.store(true, Ordering::SeqCst);
}

/// E-Stop 상태
pub fn is_estop() -> bool {
    ESTOP.load(Ordering::SeqCst)
}

/// 현재 시스템 상태 스냅샷
pub fn status() -> SystemStatusReport {
    SystemStatusReport {
        uptime_ms: Instant::now().as_millis() as u32,
        heap_used: esp_alloc::HEAP.used() as u32,
        heap_free: esp_alloc::HEAP.free() as u32,
        estop: is_estop(),
    }
}
