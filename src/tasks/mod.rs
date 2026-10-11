//! ─────────────────────────────────────────────────────────────
//! Embassy 비동기 태스크 모듈
//! ─────────────────────────────────────────────────────────────
//!
//! 각 태스크는 독립적인 async 함수로,
//! Embassy executor가 cooperative 스케줄링합니다.
//!
//! 태스크 우선순위 설계 (향후 구현):
//!   Critical  : safety_monitor, e_stop     — <10µs 응답
//!   High      : motor_control, pid_loop    — <100µs 주기
//!   Medium    : sensor_read, imu_fusion    — 1ms 주기
//!   Low       : heartbeat, logging, comms  — 100ms+ 주기

pub mod heartbeat;
pub mod wifi;

// TODO: Phase 2 태스크들
// pub mod motor;      // PWM 모터 제어 + PID 루프
// pub mod sensor;     // IMU/인코더/ADC 센서 읽기
// pub mod comms;      // UART 브릿지 (Tier 1 ↔ Tier 2 통신)
// pub mod safety;     // Watchdog, E-Stop, 과전류 감지
