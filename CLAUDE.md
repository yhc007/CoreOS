# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

CoreOS: Rust real-time robot control firmware built on a 2-tier model — **Tier 1** (Linux, pekko-rust actor system, ~10ms orchestration) commands **Tier 2** (bare-metal MCU, hard real-time motor/sensor loops). The repo holds **three independent Cargo crates** (no workspace), each for a different target:

| Crate | Path | Target | Framework | Role |
|---|---|---|---|---|
| `rtos-esp32s3` | `/` (root) | `xtensa-esp32s3-none-elf` | esp-hal + Embassy (no_std) | Tier 2 on ESP32-S3; talks to an external Linux SBC over UART/SPI |
| `rtos-am5729-m4` | `targets/am5729/cortex-m4` | `thumbv7em-none-eabihf` | RTIC v2 (no_std) | Tier 2 on the AM5729 IPU1 Cortex-M4, loaded by Linux remoteproc |
| `robot-brain-am5729` | `targets/am5729/cortex-a15` | `armv7-unknown-linux-gnueabihf` | tokio + pekko-rust (std) | Tier 1 on the same AM5729 SoC's Cortex-A15 Linux |

Docs and comments are in Korean. `BUILD_STATUS.md` tracks each target's verification status, and `targets/am5729/ARCHITECTURE.md` describes the SoC core and peripheral allocation plus the TSN design.

## Build commands

**ESP32-S3 (root)**: needs the `esp` Xtensa toolchain (`espup install`, then `source $HOME/export-esp.sh`) and `espflash`.
```bash
make build      # cargo build --release
make flash      # flash + monitor (PORT=/dev/cu.usbserial-* on macOS; default /dev/ttyUSB0)
make monitor | make check | make lint | make fmt | make size
```

**AM5729** (`targets/am5729/Makefile`):
```bash
make m4         # cd cortex-m4 && cargo build --release
make a15        # cross build for armv7-unknown-linux-gnueabihf
make deploy     # scp + remoteproc restart / systemctl restart (BOARD_IP=192.168.1.100)
```

### Build gotchas (verified from the config)
- The root `rust-toolchain.toml` (`channel = "esp"`) and `.cargo/config.toml` (`build.target = xtensa…`, `-Z build-std`) are found from subdirectories too. `cortex-m4/.cargo/config.toml` overrides only `build.target`. `cortex-a15` has no override, so always pass `--target` there (the Makefile does). Run commands from inside the crate's own directory.
- `cortex-a15` depends on pekko-rust via `path = "../../../../pekko-rust/..."`, which resolves to a sibling of the repo's parent directory. That path does not exist in this checkout, so A15 builds fail until it is cloned there or the path is edited.
- The M4 crate has no real PAC. `src/pac/*` holds hand-written register maps from TI TRM SPRUHZ6 that have not been checked on hardware. The RTIC dispatchers (IRQ 55–58) are placeholders.

### Tests
Unit tests live in `src/drivers/{pid,protocol}.rs`, which are exposed through `src/lib.rs` (`#![cfg_attr(not(test), no_std)]`) so they can build on the host. `src/main.rs` does not use the lib.
```bash
make test                          # uses the esp toolchain from rust-toolchain.toml
make test CARGO="cargo +nightly"   # if espup isn't installed (needs nightly + rust-src)
cargo +nightly test --target <host-triple> --config 'unstable.build-std=["std","test","panic_unwind"]' --lib pid_reset   # single test
```
- Code in `lib.rs` must not touch esp-hal or embassy. Those dependencies sit under `[target.'cfg(target_arch = "xtensa")'.dependencies]` and are absent on the host.
- The `--config build-std` line is required. Cargo merges config arrays rather than replacing them, so the root `build-std = ["core","alloc"]` cannot be turned off. Without the extra crates, host `std` collides with the rebuilt `core` (E0152 duplicate lang item).
- `esp-hal` is pinned to `=1.0.0-rc.0` because `esp-hal-embassy 0.9` requires esp-hal's `requires-unstable` feature, which final 1.0.0 removed. A move to esp-hal 1.x final means replacing esp-hal-embassy, and `main.rs` still uses `esp_hal::prelude`.

## Architecture notes

- **ESP32-S3 crate**: `main.rs` sets up the heap (esp-alloc, 32KB), the Embassy time driver on TIMG0 and logging, then spawns Embassy tasks from `src/tasks/`. Each peripheral pin is passed individually into a `src/hal/` driver. Phase 1 is only a heartbeat LED on GPIO2. PWM, I2C, UART and CAN are Phase 2 TODOs in `main.rs`.
- **Duplicated control code**: the PID controller exists in both `src/drivers/pid.rs` and `targets/am5729/cortex-m4/src/pid.rs`. The Tier1↔Tier2 protocol lives in `src/drivers/protocol.rs`, and M4 has its own command/feedback encoding. They are meant to move into a shared workspace crate later, so when changing one copy, check the other.
- **M4 firmware flow** (`cortex-m4/src/main.rs`, RTIC `mod app`):
  - `#[pre_init]` runs `pac::ammu::init_ammu()` before `.data`/`.bss` init, because the AMMU address translation must exist before RAM or peripheral access. Inside `pre_init`, use raw register writes only: no statics, no defmt.
  - `resource_table.rs` is the remoteproc resource table (carveouts, rpmsg vdev with 2 vrings, trace buffer). Its section placement in `memory.x` (IPU_RAM 64KB, OCMC1, IPC_SHARED 1MB) must match what Linux expects.
  - `ipc/` is mailbox → virtio vring → rpmsg (including the name-service announcement), all hand-written.
  - The loops are 6-axis PID motor control at 1kHz, sensors at 1kHz and feedback to A15 at 100Hz. Hardware interrupts are bound to Mailbox5 (rpmsg RX), DCAN1 and PRU-ICSS1 host events (TSN).
- **A15 side** (`cortex-a15/src/main.rs`): opens `/dev/rpmsg0` (`ipc.rs`) for commands and feedback with M4. ActorSystem spawning is still commented out. `actors/tsn.rs` (`TsnActor`) manages gPTP by shelling out to linuxptp (`ptp4l`/`phc2sys`) and `tc taprio` for 802.1Qbv; it needs root. The matching M4-side TSN pieces are in `pac/cpsw.rs` (CPTS timestamps, ALE) and `pac/pru.rs`.
- Logging differs per target: `log` + esp-println (`ESP_LOG`) on ESP32, `defmt` + RTT (`DEFMT_LOG`) on M4, `tracing` on A15.
