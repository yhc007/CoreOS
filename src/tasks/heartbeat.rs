//! ─────────────────────────────────────────────────────────────
//! Heartbeat Task — 시스템 생존 표시
//! ─────────────────────────────────────────────────────────────
//!
//! LED를 주기적으로 깜빡여 시스템이 정상 동작 중임을 표시합니다.
//! 패턴으로 시스템 상태를 나타냅니다:
//!   - 느린 점멸 (1Hz)  : 정상 (Idle)
//!   - 빠른 점멸 (5Hz)  : 동작 중 (Running)
//!   - 상시 켜짐         : 에러 상태
//!   - 상시 꺼짐         : 시스템 다운

use embassy_time::{Duration, Timer};
use log::info;

use crate::hal::gpio::LedDriver;

/// Heartbeat LED 점멸 패턴
#[derive(Clone, Copy, PartialEq)]
pub enum HeartbeatPattern {
    /// 정상 대기 — 1초 주기
    Idle,
    /// 동작 중 — 200ms 주기
    Running,
    /// 에러 — 상시 켜짐
    Error,
}

impl HeartbeatPattern {
    fn interval_ms(&self) -> u64 {
        match self {
            Self::Idle => 500,
            Self::Running => 100,
            Self::Error => 0, // no blink
        }
    }
}

/// Heartbeat 태스크 — Embassy async task
///
/// 이 태스크는 시스템이 살아있는 동안 계속 실행됩니다.
/// LED를 토글하며 시스템 상태를 시각적으로 표시합니다.
#[embassy_executor::task]
pub async fn heartbeat_task(mut led: LedDriver) {
    let pattern = HeartbeatPattern::Idle;
    let mut count: u64 = 0;

    info!("[HEARTBEAT] Started — pattern: Idle (1Hz)");

    loop {
        match pattern {
            HeartbeatPattern::Error => {
                led.on();
                Timer::after(Duration::from_secs(1)).await;
            }
            _ => {
                led.toggle();
                count += 1;

                if count % 20 == 0 {
                    info!("[HEARTBEAT] Blink #{}", count);
                }

                Timer::after(Duration::from_millis(pattern.interval_ms())).await;
            }
        }

        // TODO: 시스템 상태에 따라 패턴 변경
        // pattern = SYSTEM_STATE.lock().heartbeat_pattern();
    }
}
