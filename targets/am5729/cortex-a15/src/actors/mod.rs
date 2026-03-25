//! ─────────────────────────────────────────────────────────────
//! pekko-rust 로봇 액터 모듈
//! ─────────────────────────────────────────────────────────────
//!
//! AM5729 Cortex-A15 (Linux) 에서 실행되는 액터들.
//! 각 액터는 pekko-actor 기반이며,
//! rpmsg IPC를 통해 M4 RTIC 펌웨어와 통신합니다.
//!
//! 액터 트리:
//!   /robot/supervisor (OneForOneStrategy)
//!   ├── /robot/comm-bridge      → M4 IPC (rpmsg)
//!   ├── /robot/safety-monitor   → FSM (Normal/Warning/Emergency)
//!   ├── /robot/motion-planner   → 경로 계획 + 궤적 생성
//!   ├── /robot/sensor-fusion    → pekko-streams 파이프라인
//!   ├── /robot/telemetry        → pekko-event-bus 메트릭
//!   └── /robot/persistence      → pekko-persistence 이벤트 소싱

pub mod tsn;

// pub mod comm_bridge;
// pub mod safety_monitor;
// pub mod motion_planner;
// pub mod sensor_fusion;
// pub mod telemetry;
// pub mod persistence;
