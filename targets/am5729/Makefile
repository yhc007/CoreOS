# ────────────────────────────────────────────────
# AM5729 — Multi-Core Robot Build System
# ────────────────────────────────────────────────
#
# 두 타겟을 동시에 빌드합니다:
#   1. Cortex-M4 (RTIC) — thumbv7em-none-eabihf
#   2. Cortex-A15 (pekko-rust) — armv7-unknown-linux-gnueabihf
#
# 크로스 컴파일 필요:
#   sudo apt install gcc-arm-linux-gnueabihf

# AM5729 보드 IP (SSH 배포용)
BOARD_IP ?= 192.168.1.100
BOARD_USER ?= root

# M4 펌웨어 경로 (remoteproc이 읽는 위치)
M4_FW_PATH = /lib/firmware/am5729-ipu1-fw.xem4

.PHONY: all m4 a15 deploy deploy-m4 deploy-a15 clean

# ── 전체 빌드 ──
all: m4 a15

# ── Cortex-M4 RTIC 펌웨어 빌드 ──
m4:
	@echo "═══ Building Cortex-M4 IPU1 (RTIC) ═══"
	cd cortex-m4 && cargo build --release
	@echo "Binary: cortex-m4/target/thumbv7em-none-eabihf/release/rtos-am5729-m4"

# ── Cortex-A15 pekko-rust 빌드 ──
a15:
	@echo "═══ Building Cortex-A15 (pekko-rust) ═══"
	cd cortex-a15 && cargo build --release \
		--target armv7-unknown-linux-gnueabihf
	@echo "Binary: cortex-a15/target/armv7-unknown-linux-gnueabihf/release/robot-brain"

# ── 네이티브 빌드 (A15가 AM5729 보드에서 직접 빌드하는 경우) ──
a15-native:
	cd cortex-a15 && cargo build --release

# ── 배포: 전체 ──
deploy: deploy-m4 deploy-a15

# ── M4 펌웨어 배포 (remoteproc 경유) ──
deploy-m4: m4
	@echo "═══ Deploying M4 firmware to $(BOARD_IP) ═══"
	# 1. 펌웨어 전송
	scp cortex-m4/target/thumbv7em-none-eabihf/release/rtos-am5729-m4 \
		$(BOARD_USER)@$(BOARD_IP):$(M4_FW_PATH)
	# 2. M4 재시작
	ssh $(BOARD_USER)@$(BOARD_IP) \
		"echo stop > /sys/class/remoteproc/remoteproc0/state 2>/dev/null; \
		 sleep 0.1; \
		 echo start > /sys/class/remoteproc/remoteproc0/state"
	@echo "M4 firmware deployed and restarted"

# ── A15 바이너리 배포 ──
deploy-a15: a15
	@echo "═══ Deploying A15 robot-brain to $(BOARD_IP) ═══"
	scp cortex-a15/target/armv7-unknown-linux-gnueabihf/release/robot-brain \
		$(BOARD_USER)@$(BOARD_IP):/opt/robot/robot-brain
	ssh $(BOARD_USER)@$(BOARD_IP) \
		"systemctl restart robot-brain || /opt/robot/robot-brain &"
	@echo "A15 robot-brain deployed"

# ── 클린 ──
clean:
	cd cortex-m4 && cargo clean
	cd cortex-a15 && cargo clean

# ── 디버그 (CCS JTAG) ──
debug-m4:
	@echo "Launch Code Composer Studio and connect to AM5729 IPU1"
	@echo "Target binary: cortex-m4/target/thumbv7em-none-eabihf/release/rtos-am5729-m4"

# ── 초기 환경 설정 가이드 ──
setup:
	@echo "═══════════════════════════════════════════"
	@echo "  AM5729 개발 환경 설정"
	@echo "═══════════════════════════════════════════"
	@echo ""
	@echo "1. ARM 크로스 컴파일러:"
	@echo "   sudo apt install gcc-arm-linux-gnueabihf"
	@echo ""
	@echo "2. Rust 타겟 추가:"
	@echo "   rustup target add thumbv7em-none-eabihf     # M4"
	@echo "   rustup target add armv7-unknown-linux-gnueabihf  # A15"
	@echo ""
	@echo "3. TI Processor SDK (선택):"
	@echo "   https://www.ti.com/tool/PROCESSOR-SDK-AM57X"
	@echo ""
	@echo "4. AM5729 SVD → PAC 생성 (M4 레지스터):"
	@echo "   cargo install svd2rust"
	@echo "   svd2rust -i AM5729_M4.svd --target cortex-m"
	@echo ""
	@echo "5. 보드 연결:"
	@echo "   ssh root@\$(BOARD_IP)"
	@echo ""
