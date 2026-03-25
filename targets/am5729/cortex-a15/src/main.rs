//! ─────────────────────────────────────────────────────────────
//! Robot Brain — pekko-rust on AM5729 Cortex-A15
//! ─────────────────────────────────────────────────────────────
//!
//! AM5729 Cortex-A15 (Linux) 에서 실행되는 pekko-rust 기반
//! 로봇 오케스트레이션 레이어.
//!
//! 이 프로세스는 rpmsg를 통해 M4 RTIC 펌웨어와 통신합니다:
//!   /dev/rpmsg0  → 명령 전송 (A15 → M4)
//!   /dev/rpmsg0  ← 피드백 수신 (M4 → A15)

mod actors;
mod ipc;

use anyhow::Result;
use tracing::{info, warn, Level};
use tracing_subscriber::FmtSubscriber;

use crate::actors::tsn::{TsnActor, TsnConfig, TsnSyncState};

#[tokio::main]
async fn main() -> Result<()> {
    // 로깅 초기화
    let subscriber = FmtSubscriber::builder()
        .with_max_level(Level::INFO)
        .finish();
    tracing::subscriber::set_global_default(subscriber)?;

    info!("══════════════════════════════════════");
    info!("  Robot Brain — AM5729 Cortex-A15");
    info!("  pekko-rust Actor System");
    info!("══════════════════════════════════════");

    // 1. rpmsg IPC 채널 열기
    let ipc = ipc::RpmsgChannel::open("/dev/rpmsg0").await?;
    info!("[IPC] rpmsg channel opened → M4 IPU1");

    // 2. pekko-rust ActorSystem 생성
    // let system = ActorSystem::new("robot-system")?;

    // 3. 액터 스폰
    // let comm_bridge = system.spawn("comm-bridge",
    //     actors::CommBridgeActor::new(ipc))?;
    // let safety_monitor = system.spawn("safety-monitor",
    //     actors::SafetyMonitorActor::new())?;
    // let motion_planner = system.spawn("motion-planner",
    //     actors::MotionPlannerActor::new())?;
    // let sensor_fusion = system.spawn("sensor-fusion",
    //     actors::SensorFusionActor::new())?;
    // let telemetry = system.spawn("telemetry",
    //     actors::TelemetryActor::new())?;

    info!("[INIT] All actors spawned");

    // 3. TSN Actor 초기화
    info!("[TSN] Initializing TSN Actor...");
    let mut tsn_actor = TsnActor::with_defaults();
    match tsn_actor.init().await {
        Ok(()) => info!("[TSN] TSN stack initialized successfully"),
        Err(e) => warn!("[TSN] TSN init failed (may need root): {}", e),
    }

    // 4. 메인 루프 — IPC + TSN 모니터링
    let mut tsn_poll_interval = tokio::time::interval(
        std::time::Duration::from_secs(1)
    );

    loop {
        tokio::select! {
            // rpmsg IPC 수신
            msg = ipc.recv() => {
                match msg {
                    Ok(data) => {
                        info!("[IPC] Received from M4: {} bytes", data.len());
                        // 디코딩 → Actor에 전달
                    }
                    Err(e) => {
                        tracing::error!("[IPC] Error: {}", e);
                        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
                    }
                }
            }

            // TSN 상태 폴링 (1초 주기)
            _ = tsn_poll_interval.tick() => {
                let sync_state = tsn_actor.poll_sync_status();
                match sync_state {
                    TsnSyncState::Synced { offset_ns } => {
                        info!("[TSN] gPTP SYNCED — offset={}ns", offset_ns);
                    }
                    TsnSyncState::Synchronizing => {
                        info!("[TSN] gPTP synchronizing...");
                    }
                    TsnSyncState::Lost => {
                        warn!("[TSN] gPTP sync LOST!");
                    }
                    _ => {}
                }
            }
        }
    }
}
