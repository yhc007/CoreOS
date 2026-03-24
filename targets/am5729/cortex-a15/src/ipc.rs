//! ─────────────────────────────────────────────────────────────
//! rpmsg IPC Channel — A15 ↔ M4 통신
//! ─────────────────────────────────────────────────────────────
//!
//! TI remoteproc/rpmsg 프레임워크를 통한 코어 간 통신.
//!
//! Linux 측 인터페이스:
//!   - /dev/rpmsg0       : 데이터 채널
//!   - /sys/class/remoteproc/remoteproc0 : M4 제어
//!
//! 메시지 직렬화: postcard (no_std 호환 serde)

use anyhow::{Context, Result};
use bytes::BytesMut;
use std::path::Path;
use tokio::fs::{File, OpenOptions};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tracing::{debug, info, warn};

/// rpmsg 최대 메시지 크기 (TI rpmsg 기본값)
const RPMSG_BUF_SIZE: usize = 512;

/// rpmsg IPC 채널
pub struct RpmsgChannel {
    file: File,
    read_buf: BytesMut,
}

impl RpmsgChannel {
    /// rpmsg 디바이스 파일 열기
    pub async fn open(device_path: &str) -> Result<Self> {
        // remoteproc이 M4를 부팅했는지 확인
        Self::check_remoteproc_state().await?;

        let path = Path::new(device_path);
        if !path.exists() {
            anyhow::bail!(
                "rpmsg device {} not found. Is remoteproc loaded?",
                device_path
            );
        }

        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(device_path)
            .await
            .with_context(|| format!("Failed to open {}", device_path))?;

        info!("[RPMSG] Opened {}", device_path);

        Ok(Self {
            file,
            read_buf: BytesMut::with_capacity(RPMSG_BUF_SIZE),
        })
    }

    /// M4에 메시지 전송
    pub async fn send(&mut self, data: &[u8]) -> Result<()> {
        if data.len() > RPMSG_BUF_SIZE {
            anyhow::bail!(
                "Message too large: {} > {} bytes",
                data.len(),
                RPMSG_BUF_SIZE
            );
        }
        self.file.write_all(data).await?;
        debug!("[RPMSG] Sent {} bytes to M4", data.len());
        Ok(())
    }

    /// M4에서 메시지 수신 (blocking async)
    pub async fn recv(&self) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; RPMSG_BUF_SIZE];
        // Note: tokio File의 read는 epoll 기반
        // rpmsg 디바이스는 poll을 지원하므로 async 동작
        let mut file_clone = self.file.try_clone().await?;
        let n = file_clone.read(&mut buf).await?;
        buf.truncate(n);
        debug!("[RPMSG] Received {} bytes from M4", n);
        Ok(buf)
    }

    /// 직렬화된 명령 전송 (postcard)
    pub async fn send_command<T: serde::Serialize>(&mut self, cmd: &T) -> Result<()> {
        let serialized =
            postcard::to_allocvec(cmd).context("Failed to serialize command")?;
        self.send(&serialized).await
    }

    /// 직렬화된 피드백 수신 (postcard)
    pub async fn recv_feedback<T: serde::de::DeserializeOwned>(&self) -> Result<T> {
        let data = self.recv().await?;
        let feedback =
            postcard::from_bytes(&data).context("Failed to deserialize feedback")?;
        Ok(feedback)
    }

    /// remoteproc 상태 확인
    async fn check_remoteproc_state() -> Result<()> {
        let state_path = "/sys/class/remoteproc/remoteproc0/state";

        if !Path::new(state_path).exists() {
            warn!("[RPMSG] remoteproc sysfs not found — skipping check");
            return Ok(());
        }

        let state = tokio::fs::read_to_string(state_path)
            .await
            .unwrap_or_else(|_| "unknown".to_string());
        let state = state.trim();

        match state {
            "running" => {
                info!("[RPMSG] M4 IPU1 is running");
                Ok(())
            }
            "offline" => {
                info!("[RPMSG] Starting M4 IPU1 via remoteproc...");
                tokio::fs::write(state_path, "start").await?;
                // M4 부팅 대기
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
                Ok(())
            }
            _ => {
                warn!("[RPMSG] M4 IPU1 state: {}", state);
                Ok(())
            }
        }
    }

    /// M4 펌웨어 로드 (remoteproc)
    pub async fn load_firmware(firmware_path: &str) -> Result<()> {
        let fw_name_path = "/sys/class/remoteproc/remoteproc0/firmware";

        // 1. M4 중지
        let state_path = "/sys/class/remoteproc/remoteproc0/state";
        let _ = tokio::fs::write(state_path, "stop").await;
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // 2. 펌웨어 파일명 설정
        let fw_name = Path::new(firmware_path)
            .file_name()
            .context("Invalid firmware path")?
            .to_str()
            .context("Non-UTF8 firmware name")?;

        tokio::fs::write(fw_name_path, fw_name).await?;
        info!("[RPMSG] Firmware set: {}", fw_name);

        // 3. M4 시작
        tokio::fs::write(state_path, "start").await?;
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        info!("[RPMSG] M4 IPU1 started with new firmware");

        Ok(())
    }
}
