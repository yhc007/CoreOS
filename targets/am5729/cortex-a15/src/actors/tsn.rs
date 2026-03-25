//! ─────────────────────────────────────────────────────────────
//! TSN Actor — IEEE 802.1AS gPTP + linuxptp 연동
//! ─────────────────────────────────────────────────────────────
//!
//! A15 Linux에서 실행되는 TSN 관리 액터.
//!
//! 역할:
//!   1. linuxptp (ptp4l) 데몬과 연동하여 gPTP 동기화 관리
//!   2. CPSW CPTS 하드웨어 타임스탬프 모니터링
//!   3. TSN 네트워크 상태를 다른 액터에게 브로드캐스트
//!   4. EST(802.1Qbv) 스케줄 관리 (tc-taprio via netlink)
//!   5. M4 PRU-ICSS와의 시간 동기화 상태 감시
//!
//! 의존:
//!   - linuxptp (ptp4l + phc2sys): IEEE 802.1AS gPTP 구현
//!   - CPSW CPTS: /dev/ptp0 (하드웨어 PTP 클럭)
//!   - tc-taprio: Linux Traffic Control for 802.1Qbv EST
//!
//! 참고: TRM SPRUHZ6 Chapter 14 (Ethernet Subsystem)
//!       IEEE 802.1AS-2020, IEEE 802.1Qbv-2015

use std::process::Command;
use std::time::Duration;
use tracing::{info, warn, error};

/// TSN 동기화 상태
#[derive(Clone, Debug, PartialEq)]
pub enum TsnSyncState {
    /// 초기화 중 — ptp4l 시작 전
    Initializing,
    /// gPTP 동기화 진행 중
    Synchronizing,
    /// gPTP 동기화 완료 (offset < 100ns)
    Synced { offset_ns: i64 },
    /// 동기화 손실
    Lost,
    /// 오류
    Error(String),
}

/// TSN 네트워크 상태
#[derive(Clone, Debug)]
pub struct TsnNetworkStatus {
    /// gPTP 동기화 상태
    pub sync_state: TsnSyncState,
    /// Grandmaster 클럭 Identity
    pub gm_identity: [u8; 8],
    /// 현재 UTC 오프셋
    pub utc_offset: i32,
    /// 경로 지연 (ns)
    pub path_delay_ns: i64,
    /// EST 스케줄 활성 여부
    pub est_active: bool,
    /// PHY 링크 상태
    pub link_up: bool,
}

/// TSN 설정
#[derive(Clone, Debug)]
pub struct TsnConfig {
    /// 네트워크 인터페이스 이름 (예: "eth0")
    pub interface: String,
    /// PTP 하드웨어 클럭 디바이스 (예: "/dev/ptp0")
    pub ptp_device: String,
    /// gPTP 프로필 사용 여부
    pub gptp_profile: bool,
    /// EST 사이클 시간 (ns)
    pub est_cycle_ns: u32,
    /// TSN 트래픽 클래스 정의
    pub traffic_classes: Vec<TrafficClass>,
}

/// TSN 트래픽 클래스
#[derive(Clone, Debug)]
pub struct TrafficClass {
    /// 트래픽 클래스 번호 (0=최고 우선순위)
    pub tc: u8,
    /// 게이트 열림 시간 (ns)
    pub gate_time_ns: u32,
    /// 설명
    pub description: String,
}

impl Default for TsnConfig {
    fn default() -> Self {
        Self {
            interface: "eth0".to_string(),
            ptp_device: "/dev/ptp0".to_string(),
            gptp_profile: true,
            est_cycle_ns: 125_000, // 125μs
            traffic_classes: vec![
                TrafficClass {
                    tc: 0,
                    gate_time_ns: 30_000,
                    description: "Motor control (hard RT)".to_string(),
                },
                TrafficClass {
                    tc: 1,
                    gate_time_ns: 30_000,
                    description: "Sensor data (soft RT)".to_string(),
                },
                TrafficClass {
                    tc: 2,
                    gate_time_ns: 40_000,
                    description: "Vision + gPTP".to_string(),
                },
                TrafficClass {
                    tc: 7,
                    gate_time_ns: 25_000,
                    description: "Best-effort + safety".to_string(),
                },
            ],
        }
    }
}

/// TSN Actor
///
/// pekko-rust 액터로 구현될 예정.
/// 현재는 TSN 관리 로직만 포함.
pub struct TsnActor {
    config: TsnConfig,
    status: TsnNetworkStatus,
}

impl TsnActor {
    /// 새 TSN Actor 생성
    pub fn new(config: TsnConfig) -> Self {
        Self {
            config,
            status: TsnNetworkStatus {
                sync_state: TsnSyncState::Initializing,
                gm_identity: [0u8; 8],
                utc_offset: 37,
                path_delay_ns: 0,
                est_active: false,
                link_up: false,
            },
        }
    }

    /// 기본 설정으로 TSN Actor 생성
    pub fn with_defaults() -> Self {
        Self::new(TsnConfig::default())
    }

    /// TSN 스택 초기화
    ///
    /// 1. PHY 링크 확인
    /// 2. ptp4l 시작 (gPTP 동기화)
    /// 3. phc2sys 시작 (시스템 클럭 동기화)
    /// 4. tc-taprio 설정 (EST 스케줄)
    pub async fn init(&mut self) -> Result<(), String> {
        info!("[TSN] Initializing TSN stack on {}", self.config.interface);

        // 1. PHY 링크 확인
        self.check_link_status()?;

        // 2. ptp4l 시작
        self.start_ptp4l()?;

        // 3. phc2sys 시작
        self.start_phc2sys()?;

        // 4. EST 스케줄 설정
        self.configure_est()?;

        self.status.sync_state = TsnSyncState::Synchronizing;
        info!("[TSN] Stack initialized, waiting for gPTP sync...");
        Ok(())
    }

    /// PHY 링크 상태 확인
    fn check_link_status(&mut self) -> Result<(), String> {
        let output = Command::new("ip")
            .args(["link", "show", &self.config.interface])
            .output()
            .map_err(|e| format!("ip link failed: {}", e))?;

        let stdout = String::from_utf8_lossy(&output.stdout);
        self.status.link_up = stdout.contains("state UP");

        if self.status.link_up {
            info!("[TSN] {} link UP", self.config.interface);
            Ok(())
        } else {
            warn!("[TSN] {} link DOWN", self.config.interface);
            Err(format!("{} link down", self.config.interface))
        }
    }

    /// ptp4l 데몬 시작 (gPTP 프로파일)
    ///
    /// ptp4l은 IEEE 802.1AS gPTP 구현체로,
    /// CPSW CPTS 하드웨어 타임스탬프를 사용합니다.
    fn start_ptp4l(&self) -> Result<(), String> {
        let mut args = vec![
            "-i".to_string(), self.config.interface.clone(),
            "-m".to_string(), // 로그 → stdout
            "--tx_timestamp_timeout".to_string(), "40".to_string(),
        ];

        if self.config.gptp_profile {
            // gPTP 프로파일 (802.1AS)
            args.extend_from_slice(&[
                "-f".to_string(),
                "/etc/linuxptp/gPTP.cfg".to_string(),
            ]);
        }

        // CPSW CPTS 하드웨어 클럭 사용
        args.extend_from_slice(&[
            "--clock_servo".to_string(), "pi".to_string(),
        ]);

        info!("[TSN] Starting ptp4l: {:?}", args);

        // 백그라운드로 ptp4l 시작
        Command::new("ptp4l")
            .args(&args)
            .spawn()
            .map_err(|e| format!("ptp4l start failed: {}", e))?;

        info!("[TSN] ptp4l started");
        Ok(())
    }

    /// phc2sys 시작 — PTP HW 클럭 → 시스템 클럭 동기화
    fn start_phc2sys(&self) -> Result<(), String> {
        let args = [
            "-s", &self.config.ptp_device,
            "-c", "CLOCK_REALTIME",
            "-O", "0",           // UTC 오프셋 자동
            "-m",                // 로그 → stdout
            "-R", "100",         // 100Hz 폴링
        ];

        info!("[TSN] Starting phc2sys: {:?}", args);

        Command::new("phc2sys")
            .args(&args)
            .spawn()
            .map_err(|e| format!("phc2sys start failed: {}", e))?;

        info!("[TSN] phc2sys started");
        Ok(())
    }

    /// tc-taprio로 EST (802.1Qbv) 스케줄 설정
    ///
    /// Linux Traffic Control의 taprio qdisc를 사용하여
    /// CPSW EST 하드웨어에 Gate Control List를 프로그래밍합니다.
    ///
    /// 로봇 제어용 GCL (125μs 사이클):
    ///   TC0 0~30μs: 모터 제어 (최고 우선순위)
    ///   TC1 30~60μs: 센서 데이터
    ///   TC2 60~100μs: 비전 + gPTP
    ///   TC7 100~125μs: Best-effort + 안전
    fn configure_est(&mut self) -> Result<(), String> {
        // tc-taprio qdisc 설정
        // 기존 qdisc 삭제
        let _ = Command::new("tc")
            .args(["qdisc", "del", "dev", &self.config.interface, "root"])
            .output();

        // taprio 스케줄 설정
        let mut args = vec![
            "qdisc".to_string(), "add".to_string(),
            "dev".to_string(), self.config.interface.clone(),
            "parent".to_string(), "root".to_string(),
            "taprio".to_string(),
            "num_tc".to_string(), "8".to_string(),
            // TC → 큐 매핑 (prio → TC)
            "map".to_string(),
            "0".to_string(), "1".to_string(), "2".to_string(),
            "3".to_string(), "4".to_string(), "5".to_string(),
            "6".to_string(), "7".to_string(),
            // 큐 카운트@오프셋
            "queues".to_string(),
            "1@0".to_string(), "1@1".to_string(), "1@2".to_string(),
            "1@3".to_string(), "1@4".to_string(), "1@5".to_string(),
            "1@6".to_string(), "1@7".to_string(),
        ];

        // GCL 엔트리
        for tc in &self.config.traffic_classes {
            let gate_mask = 1u32 << tc.tc;
            args.push("sched-entry".to_string());
            args.push("S".to_string());
            args.push(format!("{:#04x}", gate_mask));
            args.push(tc.gate_time_ns.to_string());
        }

        // 사이클 시간
        args.push("base-time".to_string());
        args.push("0".to_string());
        args.push("cycle-time".to_string());
        args.push(self.config.est_cycle_ns.to_string());

        // 하드웨어 오프로드 모드 (CPSW EST)
        args.push("flags".to_string());
        args.push("0x2".to_string()); // TAPRIO_FLAGS_FULL_OFFLOAD

        info!("[TSN] Configuring tc-taprio: {} entries, cycle={}ns",
              self.config.traffic_classes.len(), self.config.est_cycle_ns);

        let output = Command::new("tc")
            .args(&args)
            .output()
            .map_err(|e| format!("tc taprio failed: {}", e))?;

        if output.status.success() {
            self.status.est_active = true;
            info!("[TSN] EST schedule active — {} GCL entries",
                  self.config.traffic_classes.len());
            Ok(())
        } else {
            let stderr = String::from_utf8_lossy(&output.stderr);
            error!("[TSN] tc taprio failed: {}", stderr);
            Err(format!("tc taprio failed: {}", stderr))
        }
    }

    /// gPTP 동기화 상태 업데이트 (pmc 도구 사용)
    pub fn poll_sync_status(&mut self) -> &TsnSyncState {
        // pmc (PTP Management Client)로 동기화 상태 조회
        if let Ok(output) = Command::new("pmc")
            .args(["-u", "-b", "0", "GET", "CURRENT_DATA_SET"])
            .output()
        {
            let stdout = String::from_utf8_lossy(&output.stdout);

            // offsetFromMaster 파싱
            if let Some(offset_line) = stdout.lines()
                .find(|l| l.contains("offsetFromMaster"))
            {
                if let Some(offset_str) = offset_line.split_whitespace().last() {
                    if let Ok(offset_ns) = offset_str.parse::<i64>() {
                        if offset_ns.abs() < 100 {
                            // 100ns 이내 = 동기화 완료
                            self.status.sync_state = TsnSyncState::Synced { offset_ns };
                        } else {
                            self.status.sync_state = TsnSyncState::Synchronizing;
                        }
                    }
                }
            }
        }

        &self.status.sync_state
    }

    /// 현재 TSN 네트워크 상태 반환
    pub fn network_status(&self) -> &TsnNetworkStatus {
        &self.status
    }
}

// ── pekko-rust Actor trait 구현 (향후 활성화) ──
//
// impl Actor for TsnActor {
//     type Msg = TsnMessage;
//
//     fn recv(&mut self, ctx: &Context<Self::Msg>, msg: Self::Msg, sender: Sender) {
//         match msg {
//             TsnMessage::PollSync => {
//                 let state = self.poll_sync_status();
//                 // 상태 변경 시 브로드캐스트
//                 ctx.system.event_bus().publish(TsnEvent::SyncChanged(state.clone()));
//             }
//             TsnMessage::GetStatus => {
//                 if let Some(sender) = sender {
//                     sender.tell(TsnMessage::StatusReport(self.status.clone()), None);
//                 }
//             }
//             TsnMessage::ReconfigureEst(config) => {
//                 self.config = config;
//                 self.configure_est().ok();
//             }
//         }
//     }
// }
