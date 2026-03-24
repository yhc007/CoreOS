//! ─────────────────────────────────────────────────────────────
//! RTOS ESP32-S3 — Robot Control Firmware (Phase 1)
//! ─────────────────────────────────────────────────────────────
//!
//! Embassy async runtime 기반의 실시간 로봇 제어 펌웨어.
//! 향후 pekko-rust Actor System과 2-Tier 아키텍처로 통합 예정.
//!
//! Architecture:
//!   [Tier 2: ESP32-S3 (이 펌웨어)]
//!     - Hard real-time 모터/센서 제어 (<100µs)
//!     - Embassy async tasks로 동시성 관리
//!     - UART/SPI/I2C 로 Tier 1 (Linux SBC 또는 AM5729 A15) 과 통신
//!
//!   [Tier 1: Linux SBC (pekko-rust)]
//!     - 고수준 로봇 오케스트레이션 (~10ms)
//!     - Actor System 기반 상태 관리
//!     - CoreDB 로 센서 데이터 저장

#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::prelude::*;
use esp_hal::timer::timg::TimerGroup;
use log::info;

mod hal;
mod tasks;

// ── Heap Allocator ──
const HEAP_SIZE: usize = 32 * 1024;

fn init_heap() {
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            &mut HEAP as *mut _ as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

// ────────────────────────────────────────────────
// Main Entry Point
// ────────────────────────────────────────────────

#[esp_hal::main]
async fn main(spawner: Spawner) {
    // 1. 시스템 초기화
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap();

    // 2. Embassy 타이머 드라이버 초기화 (TIMG0 → esp-hal-embassy)
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // 3. 로깅 초기화
    esp_println::logger::init_logger_from_env();
    info!("══════════════════════════════════════");
    info!("  RTOS ESP32-S3 — Robot Controller");
    info!("  Version: {}", env!("CARGO_PKG_VERSION"));
    info!("══════════════════════════════════════");

    // 4. HAL 드라이버 초기화 (개별 핀 전달)
    let led = hal::gpio::LedDriver::new(peripherals.GPIO2);
    info!("[INIT] LED driver initialized (GPIO2)");

    // TODO: Phase 2 — 각 핀을 개별 전달
    // let pwm = hal::pwm::PwmDriver::new(peripherals.MCPWM0, peripherals.GPIO4);
    // let i2c = I2c::new(peripherals.I2C0, Config::default())
    //     .with_sda(peripherals.GPIO8)
    //     .with_scl(peripherals.GPIO9);
    // let uart = Uart::new(peripherals.UART1, Config::default())
    //     .with_tx(peripherals.GPIO17)
    //     .with_rx(peripherals.GPIO18);

    // 5. Embassy 태스크 스폰
    spawner.must_spawn(tasks::heartbeat::heartbeat_task(led));
    info!("[INIT] Heartbeat task spawned");

    info!("[INIT] All tasks spawned. Entering main loop.");

    // 6. Main loop — 시스템 모니터링
    let mut tick: u64 = 0;
    loop {
        tick += 1;
        if tick % 100 == 0 {
            info!("[MAIN] System alive — tick #{}", tick);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
