# 빌드 상태 및 보드 업로드 가이드

## ESP32-S3 (esp-hal + Embassy) — ✅ 빌드 가능

### 상태
코드가 실제 ESP32-S3 보드에 플래시 가능한 수준입니다.

### 빌드 전 필요 사항
```bash
# 1. Xtensa 툴체인 설치 (1회)
cargo install espup
espup install
source $HOME/export-esp.sh

# 2. 플래싱 도구 설치 (1회)
cargo install espflash

# 3. 빌드
cd rtos-esp32s3
cargo build --release

# 4. 플래시 + 모니터
espflash flash target/xtensa-esp32s3-none-elf/release/rtos-esp32s3 --monitor
```

### 동작 내용
- LED (GPIO2) 500ms 주기로 점멸 (Heartbeat)
- 시리얼 로그 출력 (ESP_LOG=info)
- Embassy async runtime으로 태스크 스케줄링

### 알려진 제약
- esp-hal/esp-hal-embassy 버전 호환성은 실제 빌드 시 확인 필요
  (esp-hal 1.0.0 + esp-hal-embassy 0.9.x 조합)
- `esp_alloc::MemoryCapability` API가 버전에 따라 다를 수 있음
- ESP32-S3 DevKitC 기준 핀맵 (다른 보드는 GPIO 번호 조정)

---

## AM5729 Cortex-M4 (RTIC) — ⚠️ 구조 완성, PAC 검증 필요

### 상태
수동 PAC + IPC + Resource Table이 모두 구현되었습니다.
실제 빌드를 위해서는 아래 검증 단계가 필요합니다.

### 구현 완료 항목
1. **AMMU 초기화** — IPU Unicache 주소 변환 설정
   - Large Page 4개: L3_MAIN, L4_PER, DDR, L4_WKUP
   - Small Page 2개: IPU CTM, IPU INTC
   - pre_init()에서 .data/.bss보다 먼저 실행
   - 페리페럴 ID 레지스터 읽기로 매핑 검증
2. **수동 PAC** — TRM SPRUHZ6 기반 레지스터 매핑
   - GPIO (8포트), Timer (DMTIMER), UART (16550), DCAN
   - ePWM (모터 PWM, center-aligned, dead-band)
   - eQEP (인코더, quadrature, watchdog)
2. **Linker Script (memory.x)** — IPU1 메모리 맵
   - IPU_RAM 64KB, OCMC1 512KB, IPC_SHARED 1MB
   - resource_table, trace_buf 섹션 포함
3. **rpmsg IPC** — M4 ↔ A15 통신
   - Mailbox 드라이버 (Mailbox5, Sub-mailbox 4/5)
   - VirtIO Vring 구현 (descriptor, available, used ring)
   - rpmsg 프로토콜 (헤더, Name Service Announcement)
4. **remoteproc Resource Table** — Linux 호환
   - RSC_CARVEOUT (IPU RAM, IPC Shared)
   - RSC_VDEV (VirtIO rpmsg, 2x vring)
   - RSC_TRACE (디버그 트레이스 버퍼)
5. **RTIC main.rs 통합**
   - PAC 드라이버로 하드웨어 초기화
   - rpmsg 채널 + 엔드포인트 등록
   - PID 컨트롤러 6축
   - 명령/피드백 바이너리 인코딩/디코딩
   - 모터 제어 루프 1kHz, 센서 읽기 1kHz, 피드백 전송 100Hz

### 빌드하려면
```bash
# 1. Rust 타겟 추가
rustup target add thumbv7em-none-eabihf

# 2. 빌드 (device = cortex_m 플레이스홀더 사용)
cd targets/am5729/cortex-m4
cargo build --release

# 3. remoteproc 펌웨어로 복사
cp target/thumbv7em-none-eabihf/release/rtos-am5729-m4 \
   /lib/firmware/am5729-ipu1-fw.xem4

# 4. remoteproc으로 M4 로드
echo am5729-ipu1-fw.xem4 > /sys/class/remoteproc/remoteproc0/firmware
echo start > /sys/class/remoteproc/remoteproc0/state
```

### 남은 검증 작업
1. **RTIC dispatchers** — `SWI0~3`을 실제 AM5729 IPU 인터럽트로 매핑 필요
   - IPU INTC에서 사용 가능한 SW 인터럽트 번호 확인
2. ~~**AMMU 설정**~~ → ✅ 완료 (pre_init + verify_ammu_mappings)
3. **I2C 드라이버** — IMU 센서 통신용 (아직 미구현)
4. **CAN 프레임 수신** — DCAN 수신 메시지 오브젝트 설정
5. **실 보드 테스트** — 레지스터 주소/비트 필드 TRM 대조 확인

### 대안: TI-RTOS + Rust FFI
수동 PAC 대신 TI-RTOS SDK 위에 Rust 레이어를 올리는 방식도 가능:
```
[TI-RTOS C 기반] → [Rust FFI 바인딩] → [로봇 제어 로직 (Rust)]
```

---

## AM5729 Cortex-A15 (pekko-rust Linux) — ⚠️ pekko-rust 경로 필요

### 상태
pekko-rust 크레이트 경로를 실제 위치로 설정하면 빌드 가능합니다.

### 빌드하려면
```bash
# 1. ARM Linux 크로스 컴파일러
sudo apt install gcc-arm-linux-gnueabihf

# 2. Rust 타겟 추가
rustup target add armv7-unknown-linux-gnueabihf

# 3. Cargo.toml의 pekko-rust 경로를 실제 경로로 수정
#    또는 AM5729 보드에서 직접 네이티브 빌드

# 4. 크로스 빌드
cd targets/am5729/cortex-a15
cargo build --release --target armv7-unknown-linux-gnueabihf

# 5. 보드에 배포
scp target/armv7-unknown-linux-gnueabihf/release/robot-brain root@AM5729:/opt/robot/
```
