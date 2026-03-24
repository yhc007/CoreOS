# RTOS ESP32-S3 — Robot Controller

ESP32-S3 기반 실시간 로봇 제어 펌웨어. Embassy async runtime 사용.

## 아키텍처

```
┌─────────────────────────────┐
│  Tier 1: Linux SBC          │
│  pekko-rust Actor System    │
│  오케스트레이션 (~10ms)       │
└──────────┬──────────────────┘
           │ UART/SPI (< 1ms)
┌──────────▼──────────────────┐
│  Tier 2: ESP32-S3 (이 코드)  │
│  Embassy async runtime      │
│  실시간 제어 (< 100µs)       │
└─────────────────────────────┘
```

## 환경 설정

```bash
# 1. Xtensa 툴체인 설치
cargo install espup
espup install

# 2. 플래싱 도구 설치
cargo install espflash

# 3. 환경 변수 로드
source $HOME/export-esp.sh

# 4. 빌드 테스트
make build
```

## 빌드 & 플래시

```bash
make build          # 릴리즈 빌드
make flash          # 빌드 + 플래시 + 모니터
make monitor        # 시리얼 모니터만
make test           # 호스트 유닛 테스트 (PID, 프로토콜)
```

## 프로젝트 구조

```
src/
├── main.rs              # 엔트리포인트, Embassy 태스크 스폰
├── hal/
│   ├── mod.rs           # BoardPeripherals (하드웨어 추상화)
│   └── gpio.rs          # LED 드라이버
├── tasks/
│   ├── mod.rs           # 태스크 모듈 정의
│   └── heartbeat.rs     # Heartbeat LED 태스크
└── drivers/
    ├── mod.rs           # 드라이버 모듈 정의
    ├── pid.rs           # PID 컨트롤러 (no_std)
    └── protocol.rs      # Tier1↔Tier2 통신 프로토콜
```

## Phase 로드맵

- **Phase 1** (현재): 환경 설정, 기본 HAL, LED blink
- **Phase 2**: 모터 PWM, IMU I2C, CAN Bus, PID 루프
- **Phase 3**: pekko-rust 통신 브릿지, 센서 스트림
- **Phase 4**: Safety (Watchdog, E-Stop), HIL 테스트
