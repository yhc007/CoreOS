# ────────────────────────────────────────────────
# RTOS ESP32-S3 / ESP32 — Build & Flash Makefile
# ────────────────────────────────────────────────

# USB 포트 (macOS: /dev/cu.usbserial-*, Linux: /dev/ttyUSB0)
PORT ?= /dev/ttyUSB0
BAUD ?= 921600

# 칩 선택: esp32s3 (기본) | esp32
CHIP ?= esp32s3
TARGET := xtensa-$(CHIP)-none-elf
CHIP_FLAGS := --target $(TARGET) --no-default-features --features $(CHIP)
BIN := target/$(TARGET)/release/rtos-esp32s3

.PHONY: all build flash monitor clean setup check test

# ── 기본: 빌드 후 플래시 ──
all: build flash

# ── 빌드 ──
build:
	cargo build --release $(CHIP_FLAGS)

build-debug:
	cargo build $(CHIP_FLAGS)

# ── 플래시 + 모니터 ──
flash:
	espflash flash $(BIN) \
		--port $(PORT) --baud $(BAUD) --monitor

# ── 시리얼 모니터만 ──
monitor:
	espflash monitor --port $(PORT) --baud 115200

# ── 코드 검사 ──
check:
	cargo check $(CHIP_FLAGS)

# ── 포맷 ──
fmt:
	cargo fmt

# ── Clippy 린트 ──
lint:
	cargo clippy $(CHIP_FLAGS) -- -W clippy::all

# ── 호스트 테스트 (PID, 프로토콜 등 — src/lib.rs) ──
# .cargo/config.toml 의 build-std(core, alloc)는 덮어쓸 수 없고 병합되므로
# 호스트 std/test 도 소스에서 빌드한다 (rust-src 필요).
# esp 툴체인이 없으면: make test CARGO="cargo +nightly"
CARGO ?= cargo
HOST_TARGET := $(shell $(subst cargo,rustc,$(CARGO)) -vV | sed -n 's/^host: //p')

test:
	$(CARGO) test --target $(HOST_TARGET) \
		--config 'unstable.build-std=["std","test","panic_unwind"]' \
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
	cargo size --release $(CHIP_FLAGS) -- -A
