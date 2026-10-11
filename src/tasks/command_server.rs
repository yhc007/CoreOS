//! ─────────────────────────────────────────────────────────────
//! Command Server Task — Tier 1 ↔ Tier 2 TCP 명령 서버
//! ─────────────────────────────────────────────────────────────
//!
//! Wi-Fi(STA) 위에서 TCP 포트 7000으로 `drivers::protocol` 프레임을 주고받습니다.
//! 한 번에 한 클라이언트만 처리하며, 연결이 끊기면 다시 accept 합니다.
//!
//! 지원 명령:
//!   Ping          → Pong (페이로드 에코)
//!   RequestStatus → SystemStatus
//!   EmergencyStop → E-Stop 래치 후 SystemStatus
//!   그 외 정의된 명령 → ErrorAlert(Unsupported)   — 모터/서보 HW 미연결
//!   알 수 없는 명령   → ErrorAlert(UnknownCommand)
//!   잘못된 프레임     → ErrorAlert(BadFrame) 후 연결 종료 (스트림 동기 상실)

use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_time::Duration;
use embedded_io_async::{Read, Write};
use heapless::Vec;
use log::{info, warn};
use rtos_esp32s3::drivers::protocol::{
    error_alert_payload, frame_len, CommandType, ErrorCode, Frame, ReportType, HEADER_LEN,
    MAX_FRAME,
};

use crate::system;

/// 명령 서버 TCP 포트
pub const PORT: u16 = 7000;
/// 유휴 연결 타임아웃
const IDLE_TIMEOUT: Duration = Duration::from_secs(60);

#[embassy_executor::task]
pub async fn command_server_task(stack: Stack<'static>) {
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];

    stack.wait_config_up().await;
    info!("[CMD] Listening on TCP port {}", PORT);

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(IDLE_TIMEOUT));

        if let Err(e) = socket.accept(PORT).await {
            warn!("[CMD] accept failed: {:?}", e);
            continue;
        }
        info!("[CMD] Client connected: {:?}", socket.remote_endpoint());

        match serve(&mut socket).await {
            Ok(()) => info!("[CMD] Client disconnected"),
            Err(reason) => warn!("[CMD] Connection closed: {}", reason),
        }
        socket.close();
        let _ = socket.flush().await;
    }
}

/// 한 클라이언트와의 세션 — 연결이 끊기거나 프레임 동기를 잃으면 반환
async fn serve(socket: &mut TcpSocket<'_>) -> Result<(), &'static str> {
    let mut buf = [0u8; MAX_FRAME];

    loop {
        let mut header = [0u8; HEADER_LEN];
        match socket.read_exact(&mut header).await {
            Ok(()) => {}
            Err(embedded_io_async::ReadExactError::UnexpectedEof) => return Ok(()),
            Err(_) => return Err("read error"),
        }

        let total = match frame_len(&header) {
            Ok(n) => n,
            Err(e) => {
                warn!("[CMD] Bad header: {:?}", e);
                send(socket, ReportType::ErrorAlert, &error_alert_payload(ErrorCode::BadFrame, header[3])).await?;
                return Err("bad frame header");
            }
        };

        buf[..HEADER_LEN].copy_from_slice(&header);
        socket
            .read_exact(&mut buf[HEADER_LEN..total])
            .await
            .map_err(|_| "read error")?;

        let frame = match Frame::deserialize(&buf[..total]) {
            Ok(f) => f,
            Err(e) => {
                warn!("[CMD] Bad frame: {:?}", e);
                send(socket, ReportType::ErrorAlert, &error_alert_payload(ErrorCode::BadFrame, header[3])).await?;
                return Err("bad frame");
            }
        };

        handle(socket, &frame).await?;
    }
}

/// 명령 1건 처리 후 응답 전송
async fn handle(socket: &mut TcpSocket<'_>, frame: &Frame) -> Result<(), &'static str> {
    match CommandType::from_u8(frame.msg_type) {
        Some(CommandType::Ping) => send(socket, ReportType::Pong, &frame.payload).await,
        Some(CommandType::RequestStatus) => {
            send(socket, ReportType::SystemStatus, &system::status().to_payload()).await
        }
        Some(CommandType::EmergencyStop) => {
            system::trigger_estop();
            warn!("[CMD] EMERGENCY STOP triggered (latched until reboot)");
            send(socket, ReportType::SystemStatus, &system::status().to_payload()).await
        }
        Some(cmd) => {
            warn!("[CMD] Unsupported command: {:?}", cmd);
            send(socket, ReportType::ErrorAlert, &error_alert_payload(ErrorCode::Unsupported, frame.msg_type)).await
        }
        None => {
            warn!("[CMD] Unknown command: 0x{:02x}", frame.msg_type);
            send(socket, ReportType::ErrorAlert, &error_alert_payload(ErrorCode::UnknownCommand, frame.msg_type)).await
        }
    }
}

/// 리포트 프레임 직렬화 후 전송
async fn send(socket: &mut TcpSocket<'_>, report: ReportType, payload: &[u8]) -> Result<(), &'static str> {
    let mut frame = Frame::new(report as u8);
    frame
        .payload
        .extend_from_slice(payload)
        .map_err(|_| "payload too large")?;

    let mut out: Vec<u8, 512> = Vec::new();
    frame.serialize(&mut out).map_err(|_| "serialize failed")?;
    socket.write_all(&out).await.map_err(|_| "write error")?;
    socket.flush().await.map_err(|_| "flush error")
}
