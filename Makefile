# ────────────────────────────────────────────────
# RTOS ESP32-S3 — Build & Flash Makefile
# ────────────────────────────────────────────────

# USB 포트 (macOS: /dev/cu.usbserial-*, Linux: /dev/ttyUSB0)
PORT ?= /dev/ttyUSB0
BAUD ?= 921600

.PHONY: all build flash monitor clean setup check test

# ── 기본: 빌드 후 플래시 ──
all: build flash

# ── 빌드 ──
build:
	cargo build --release

build-debug:
	cargo build

# ── 플래시 + 모니터 ──
flash:
	espflash flash target/xtensa-esp32s3-none-elf/release/rtos-esp32s3 \
		--port $(PORT) --baud $(BAUD) --monitor

# ── 시리얼 모니터만 ──
monitor:
	espflash monitor --port $(PORT) --baud 115200

# ── 코드 검사 ──
check:
	cargo check

# ── 포맷 ──
fmt:
	cargo fmt

# ── Clippy 린트 ──
lint:
	cargo clippy -- -W clippy::all

# ── 호스트 테스트 (PID, 프로토콜 등) ──
test:
	cargo test --target x86_64-unknown-linux-gnu \
		--lib -- --nocapture

# ── 클린 ──
clean:
	cargo clean

# ── 초기 환경 설정 ──
setup:
	@echo "═══════════════════════════════════════════"
	@echo "  ESP32-S3 Rust 개발 환경 설정"
	@echo "═══════════════════════════════════════════"
	@echo ""
	@echo "1. espup 설치 (Xtensa 툴체인):"
	@echo "   cargo install espup"
	@echo "   espup install"
	@echo ""
	@echo "2. espflash 설치 (플래싱 도구):"
	@echo "   cargo install espflash cargo-espflash"
	@echo ""
	@echo "3. USB 권한 설정 (Linux):"
	@echo "   sudo usermod -aG dialout $$USER"
	@echo ""
	@echo "4. 환경 변수 로드:"
	@echo "   source $$HOME/export-esp.sh"
	@echo ""
	@echo "5. 빌드 테스트:"
	@echo "   make build"
	@echo ""

# ── 칩 정보 확인 ──
info:
	espflash board-info --port $(PORT)

# ── 메모리 사용량 ──
size:
	cargo size --release -- -A
