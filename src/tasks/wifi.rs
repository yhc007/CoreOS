//! ─────────────────────────────────────────────────────────────
//! Wi-Fi Task — STA 접속 + DHCP, 또는 AP 스캔
//! ─────────────────────────────────────────────────────────────
//!
//! 빌드 시 환경 변수로 동작 모드가 결정됩니다:
//!   - WIFI_SSID (+ WIFI_PASSWORD) 지정 : 해당 AP에 접속, DHCP로 IP 획득
//!   - 미지정                           : 주변 AP를 주기적으로 스캔해 로그 출력
//!
//! 예) WIFI_SSID=MyAP WIFI_PASSWORD=secret make build CHIP=esp32
//!
//! STA 모드에서는 Tier 1 ↔ Tier 2 TCP 명령 서버(`command_server`)도 함께 시작.

use alloc::string::String;
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::WIFI;
use esp_hal::rng::Rng;
use esp_wifi::wifi::{
    self, ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState,
};
use esp_wifi::{EspWifiController, EspWifiTimerSource};
use log::{info, warn};
use static_cell::StaticCell;

/// 접속할 AP (빌드 시 주입, 없으면 스캔 모드)
const SSID: Option<&str> = option_env!("WIFI_SSID");
const PASSWORD: &str = match option_env!("WIFI_PASSWORD") {
    Some(p) => p,
    None => "",
};

/// 스캔 모드에서 한 번에 보고할 최대 AP 수
const SCAN_MAX_APS: usize = 10;
/// 스캔 / 재접속 주기
const SCAN_INTERVAL: Duration = Duration::from_secs(30);
const RECONNECT_DELAY: Duration = Duration::from_secs(5);

static WIFI_INIT: StaticCell<EspWifiController<'static>> = StaticCell::new();
static NET_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

/// Wi-Fi 초기화 후 모드에 맞는 태스크들을 스폰
///
/// 실패해도 패닉하지 않고 경고만 남깁니다 (제어 루프는 Wi-Fi 없이 동작해야 함).
pub fn start(
    spawner: &Spawner,
    timer: impl EspWifiTimerSource + 'static,
    mut rng: Rng,
    wifi_peripheral: WIFI<'static>,
) {
    let seed = (u64::from(rng.random()) << 32) | u64::from(rng.random());

    let init = match esp_wifi::init(timer, rng) {
        Ok(init) => WIFI_INIT.init(init),
        Err(e) => {
            warn!("[WIFI] init failed: {:?}", e);
            return;
        }
    };

    let (controller, interfaces) = match wifi::new(init, wifi_peripheral) {
        Ok(v) => v,
        Err(e) => {
            warn!("[WIFI] controller creation failed: {:?}", e);
            return;
        }
    };

    match SSID {
        None => {
            info!("[WIFI] WIFI_SSID not set — scan mode");
            spawner.must_spawn(scan_task(controller));
        }
        Some(ssid) => {
            info!("[WIFI] STA mode — SSID: {}", ssid);
            let (stack, runner) = embassy_net::new(
                interfaces.sta,
                embassy_net::Config::dhcpv4(Default::default()),
                NET_RESOURCES.init(StackResources::new()),
                seed,
            );
            spawner.must_spawn(connection_task(controller, ssid));
            spawner.must_spawn(net_task(runner));
            spawner.must_spawn(ip_report_task(stack));
            spawner.must_spawn(super::command_server::command_server_task(stack));
        }
    }
}

/// 주변 AP 스캔 — SCAN_INTERVAL 마다 반복
#[embassy_executor::task]
async fn scan_task(mut controller: WifiController<'static>) {
    if let Err(e) = controller.set_configuration(&Configuration::Client(Default::default())) {
        warn!("[WIFI] set_configuration failed: {:?}", e);
        return;
    }
    if let Err(e) = controller.start_async().await {
        warn!("[WIFI] start failed: {:?}", e);
        return;
    }
    log_started();

    loop {
        match controller.scan_n_async(SCAN_MAX_APS).await {
            Ok(aps) => {
                info!("[WIFI] Scan: {} AP(s)", aps.len());
                for ap in aps.iter() {
                    info!(
                        "[WIFI]   {:<32} ch {:>2}  {:>4} dBm  {:?}",
                        ap.ssid, ap.channel, ap.signal_strength, ap.auth_method
                    );
                }
            }
            Err(e) => warn!("[WIFI] scan failed: {:?}", e),
        }
        Timer::after(SCAN_INTERVAL).await;
    }
}

/// STA 접속 유지 — 끊기면 RECONNECT_DELAY 후 재접속
#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>, ssid: &'static str) {
    loop {
        if wifi::wifi_state() == WifiState::StaConnected {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            warn!("[WIFI] Disconnected — retrying in {}s", RECONNECT_DELAY.as_secs());
            Timer::after(RECONNECT_DELAY).await;
        }

        if !matches!(controller.is_started(), Ok(true)) {
            let config = Configuration::Client(ClientConfiguration {
                ssid: String::from(ssid),
                password: String::from(PASSWORD),
                ..Default::default()
            });
            if let Err(e) = controller.set_configuration(&config) {
                warn!("[WIFI] set_configuration failed: {:?}", e);
                return;
            }
            if let Err(e) = controller.start_async().await {
                warn!("[WIFI] start failed: {:?}", e);
                return;
            }
            log_started();
        }

        match controller.connect_async().await {
            Ok(()) => info!("[WIFI] Connected to {}", ssid),
            Err(e) => {
                warn!("[WIFI] connect failed: {:?} — retrying in {}s", e, RECONNECT_DELAY.as_secs());
                Timer::after(RECONNECT_DELAY).await;
            }
        }
    }
}

/// embassy-net 스택 구동
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

/// DHCP로 IP를 받으면 로그 출력
#[embassy_executor::task]
async fn ip_report_task(stack: Stack<'static>) {
    loop {
        stack.wait_config_up().await;
        if let Some(cfg) = stack.config_v4() {
            info!("[NET] IP: {}  gateway: {:?}", cfg.address, cfg.gateway);
        }
        stack.wait_config_down().await;
        warn!("[NET] Link down");
    }
}

fn log_started() {
    let mut m = [0u8; 6];
    wifi::sta_mac(&mut m);
    info!(
        "[WIFI] Started (MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x})",
        m[0], m[1], m[2], m[3], m[4], m[5]
    );
}
