# TI Sitara AM5729 — 멀티코어 로봇 제어 아키텍처

## SoC 코어 구성

```
┌─────────────────────────────────────────────────────────────────────┐
│                    TI Sitara AM5729 SoC                             │
│                                                                     │
│  ┌─────────────────────────┐  ┌─────────────────────────┐          │
│  │  ARM Cortex-A15 × 2     │  │  ARM Cortex-M4 × 2      │          │
│  │  (MPU Subsystem)        │  │  (IPU1 Subsystem)        │          │
│  │                         │  │                          │          │
│  │  Linux + pekko-rust     │  │  RTIC 펌웨어              │          │
│  │  Tier 1: 오케스트레이션   │  │  Tier 2: 실시간 제어      │          │
│  │  armv7-linux-gnueabihf  │  │  thumbv7em-none-eabihf   │          │
│  └────────────┬────────────┘  └────────────┬─────────────┘          │
│               │          IPC (rpmsg)        │                       │
│               └─────────────┬───────────────┘                       │
│                             │                                       │
│  ┌─────────────────────────┐│ ┌──────────────────────────┐         │
│  │  C66x DSP × 2           ││ │  PRU-ICSS × 2            │         │
│  │  센서 융합, FFT           ││ │  EtherCAT, 결정적 I/O     │         │
│  │  칼만 필터 가속           ││ │  나노초 정밀 GPIO          │         │
│  └─────────────────────────┘│ └──────────────────────────┘         │
│                             │                                       │
│  ┌──────────────────────────┴──────────────────────────┐           │
│  │  EVE × 4 (Embedded Vision Engine)                    │           │
│  │  딥러닝 추론, 비전 프로세싱                             │           │
│  └─────────────────────────────────────────────────────┘           │
└─────────────────────────────────────────────────────────────────────┘
```

## Rust 타겟 매핑

| 코어 | Rust 타겟 | 프레임워크 | 역할 |
|------|-----------|-----------|------|
| Cortex-A15 ×2 | `armv7-unknown-linux-gnueabihf` | pekko-rust (std) | 고수준 제어, Actor System |
| Cortex-M4 ×2 (IPU1) | `thumbv7em-none-eabihf` | RTIC v2 | 하드 RT 모터/센서 제어 |
| C66x DSP ×2 | C/Assembly (TI CGT) | TI-RTOS | 센서 융합, 신호처리 |
| PRU-ICSS ×2 | C (TI PRU CGT) | Bare-metal | EtherCAT, 결정적 I/O |
| EVE ×4 | C (TI ARP32 CGT) | TI Vision SDK | 비전/딥러닝 |

## 핵심 장점: 단일 SoC 2-Tier

ESP32-S3 방식은 별도 Linux SBC가 필요했지만, AM5729에서는:

- **Tier 1** (Cortex-A15): Linux + pekko-rust → 같은 칩
- **Tier 2** (Cortex-M4 IPU): RTIC 펌웨어 → 같은 칩
- **통신**: 외부 UART 대신 **rpmsg** (공유 메모리 IPC) → μs 수준 지연

```
ESP32-S3 방식:          AM5729 방식:
[Linux SBC] ←UART→ [MCU]    [A15 Linux] ←rpmsg→ [M4 RTIC] (같은 칩!)
   ~1ms 지연                     ~10μs 지연
```

## IPC (Inter-Processor Communication) 구조

```
Cortex-A15 (Linux)                    Cortex-M4 (RTIC)
┌──────────────┐                     ┌──────────────┐
│  pekko-rust  │                     │  RTIC Tasks  │
│  CommBridge  │                     │  Motor/Sensor│
│  Actor       │                     │              │
└──────┬───────┘                     └──────┬───────┘
       │ /dev/rpmsg0                        │ rpmsg_lite
       │                                    │
┌──────┴───────┐                     ┌──────┴───────┐
│  remoteproc  │  ←── 공유 메모리 ──→  │  IPC Driver  │
│  rpmsg       │     (DDR/OCMC)      │  (mailbox)   │
└──────────────┘                     └──────────────┘
```

### rpmsg 메시지 흐름

1. A15 (pekko-rust) → rpmsg endpoint 에 메시지 write
2. Mailbox 인터럽트 → M4 (RTIC) 에 알림
3. M4 공유 메모리에서 메시지 read → RTIC task 실행
4. M4 응답 → rpmsg → A15 pekko-rust Actor에 전달

## 페리페럴 할당 계획

| 페리페럴 | 할당 코어 | 용도 |
|---------|----------|------|
| UART1 | A15 (Linux) | 디버그 콘솔 |
| UART3 | M4 (RTIC) | 외부 센서 |
| I2C1 | M4 (RTIC) | IMU (MPU6050) |
| I2C2 | A15 (Linux) | 기타 I2C 디바이스 |
| SPI1 | M4 (RTIC) | 고속 ADC |
| McSPI2 | DSP | 센서 버퍼 |
| DCAN1 | M4 (RTIC) | CAN Bus (모터 드라이버) |
| DCAN2 | M4 (RTIC) | CAN Bus (센서 네트워크) |
| eQEP1/2 | M4 (RTIC) | 로터리 인코더 |
| ePWM1~6 | M4 (RTIC) | 모터 PWM |
| PRU-ICSS | PRU | EtherCAT / 결정적 I/O |
| Ethernet | A15 (Linux) | 네트워크, ROS2 |
| USB | A15 (Linux) | 카메라, 디바이스 |
| GPU | A15 (Linux) | UI / 시각화 |

## IPU1 AMMU (Address Memory Management Unit)

IPU의 Cortex-M4는 Unicache AMMU를 통해 외부 버스에 접근합니다.
AMMU가 없으면 GPIO, UART 등 모든 페리페럴에 접근할 수 없으므로
**부팅 시 가장 먼저 설정해야** 합니다.

### 매핑 테이블

```
┌──────────┬─────────────────────────────────────────────────────────┐
│ Page     │ IPU 논리 주소     → 물리 주소         │ 용도 (캐시)    │
├──────────┼─────────────────────────────────────────────────────────┤
│ Large 0  │ 0x4000_0000 → 0x4000_0000 (512MB)   │ L3_MAIN/OCMC   │
│          │   └ OCMC RAM1 @ 0x4030_0000          │ (WB + Prefetch)│
├──────────┼─────────────────────────────────────────────────────────┤
│ Large 1  │ 0x4800_0000 → 0x4800_0000 (512MB)   │ L4_PER 페리페럴│
│          │   └ GPIO, UART, Timer, DCAN          │ (Non-Cacheable)│
│          │   └ ePWM, eQEP, Mailbox              │                │
├──────────┼─────────────────────────────────────────────────────────┤
│ Large 2  │ 0x8000_0000 → 0x8000_0000 (512MB)   │ DDR + IPC      │
│          │   └ rpmsg 공유 메모리 @ 0x9F00_0000  │ (NC + Posted)  │
├──────────┼─────────────────────────────────────────────────────────┤
│ Large 3  │ 0x4A00_0000 → 0x4A00_0000 (512MB)   │ L4_WKUP/CFG    │
│          │   └ GPIO1 @ 0x4AE1_0000              │ (Non-Cacheable)│
│          │   └ PRM, CM_CORE 시스템 설정         │                │
├──────────┼─────────────────────────────────────────────────────────┤
│ Small 0  │ 0x0004_0000 → 0x5504_0000 (16KB)    │ IPU CTM        │
│ Small 1  │ 0x000E_0000 → 0x550E_0000 (16KB)    │ IPU INTC       │
└──────────┴─────────────────────────────────────────────────────────┘
```

### 초기화 순서

1. `pre_init()` → `ammu::init_ammu()` (cortex-m-rt가 .data/.bss보다 먼저 호출)
2. `init()` → `ammu::verify_ammu_mappings()` (페리페럴 ID 레지스터로 검증)
3. 이후 GPIO, UART, Timer 등 페리페럴 초기화 진행

### 캐시 정책 원칙

- **페리페럴 레지스터**: 반드시 Non-Cacheable (캐시하면 레지스터 읽기 지연)
- **IPC 공유 메모리**: Non-Cacheable + Posted (A15와 M4 동시 접근, 코히런시 문제)
- **코드/데이터 (OCMC)**: Write-back + Prefetch (성능 최적)
- **DDR 대용량 버퍼**: 용도에 따라 선택 (DMA는 Non-Cacheable 권장)

## IPU1 Crossbar & NVIC — RTIC Dispatcher 매핑

AM5729 IPU1은 INTC Crossbar를 통해 400+ 시스템 인터럽트를 64개 NVIC IRQ에
동적으로 매핑합니다. RTIC v2의 dispatchers에는 "사용하지 않는 IRQ"가 필요합니다.

### IRQ 할당 테이블

```
┌──────┬──────────────────────────┬──────────────────────────────┐
│ IRQ# │ Crossbar 시스템 IRQ       │ 용도                         │
├──────┼──────────────────────────┼──────────────────────────────┤
│  23  │ TIMER2_OVF (36)          │ GP Timer2 overflow           │
│  26  │ MAILBOX5_USER2 (255)     │ rpmsg 수신 (binds)           │
│  33  │ EPWM0_TZ (381)           │ 모터 비상정지 (Trip-Zone)    │
├──────┼──────────────────────────┼──────────────────────────────┤
│  55  │ RESERVED (0)             │ RTIC Dispatch — priority 1   │
│  56  │ RESERVED (0)             │ RTIC Dispatch — priority 2   │
│  57  │ RESERVED (0)             │ RTIC Dispatch — priority 3   │
│  58  │ RESERVED (0)             │ RTIC Dispatch — priority 4   │
└──────┴──────────────────────────┴──────────────────────────────┘
```

### RTIC Priority → 태스크 매핑

```
Priority 4 (최고): motor_control_loop  — 1kHz, <100μs
Priority 3:        sensor_read_loop    — 1kHz IMU I2C
Priority 2:        feedback_sender     — 10ms rpmsg TX
                   rpmsg_receive       — Mailbox5 HW IRQ (binds)
Priority 1 (최저): heartbeat           — 500ms LED
```

### 초기화 순서

1. `pre_init()` → AMMU 설정 (Crossbar 레지스터 접근 전에 필수)
2. `init()` → `intc::init_crossbar_and_nvic()`:
   - Crossbar에 HW 인터럽트 소스 매핑 (Mailbox, Timer, ePWM TZ)
   - Dispatcher IRQ들은 RESERVED(0)으로 유지
   - NVIC에서 HW 인터럽트 unmask + priority 설정

## I2C + IMU 센서

I2C1 (0x4807_0000)으로 IMU 센서와 Fast Mode (400kHz) 통신.

### 지원 센서

| 센서 | I2C 주소 | WHO_AM_I | 특징 |
|------|---------|----------|------|
| MPU-6050 | 0x68/0x69 | 0x68 | 6축, 1kHz, 저가 |
| ICM-42688-P | 0x68/0x69 | 0x47 | 6축, 32kHz, 고정밀 |
| BNO055 | 0x28/0x29 | 0xA0 | 9축 + 센서 퓨전 |

### I2C 클럭 계산

```
FUNCCLK = 96 MHz (L4_PER DPLL)
PSC = 7  → 내부 클럭 = 96MHz / 8 = 12 MHz
Fast Mode 400kHz:
  SCLL = 8  → tLOW  = (8+7)/12MHz = 1.25μs
  SCLH = 10 → tHIGH = (10+5)/12MHz = 1.25μs
  SCL period = 2.5μs → 400kHz
```

### IMU 데이터 흐름

```
sensor_read_loop (1kHz, priority 3)
  │
  ├→ I2C1 burst read (14 bytes, ~40μs @ 400kHz)
  │   └→ MPU6050: [Ax,Ay,Az,Temp,Gx,Gy,Gz] from 0x3B
  │   └→ ICM42688: [Temp,Ax,Ay,Az,Gx,Gy,Gz] from 0x1D
  │
  ├→ 매 10ms (100Hz): feedback_queue에 ImuData 전송
  │
  └→ feedback_sender → rpmsg → A15 pekko-rust
```

## DCAN (Controller Area Network) — 모터/센서 버스

DCAN1 (0x481C_C000)으로 CAN Bus를 통해 모터 드라이버 및 센서 네트워크와 통신.
AM5729 DCAN은 64개의 Message Object (MsgObj)를 사용한 하드웨어 필터링 지원.

### CAN 로봇 프로토콜 ID 체계

```
Standard CAN ID (11-bit) 할당:
  0x080         Heartbeat (노드 생존 확인)
  0x100 + id    Motor Command (모터 ID별 명령)
  0x180 + id    Motor Feedback (모터 ID별 피드백: RPM + 전류)
  0x200 + id    Servo Command (서보 모터 명령)
  0x280 + id    Sensor Data (센서 노드 데이터)
  0x7FF         Emergency Stop (전체 비상 정지)
```

### Message Object 할당 테이블

```
┌───────────────┬────────────────────────┬────────────────────────────┐
│ MsgObj 범위    │ 방향 / 용도             │ CAN ID / 필터             │
├───────────────┼────────────────────────┼────────────────────────────┤
│  1 ~ 8        │ TX: Motor Command      │ 0x100 + motor_id          │
│  9 ~ 16       │ TX: Servo Command      │ 0x200 + servo_id          │
│ 17            │ TX: Emergency Stop     │ 0x7FF                     │
│ 18 ~ 24       │ TX: 예비               │ (미사용)                   │
├───────────────┼────────────────────────┼────────────────────────────┤
│ 25 ~ 32       │ RX: Motor Feedback     │ 0x180~0x187 (Exact Match) │
│               │   IRQ ON → can_receive │ → MotorStatus 파싱         │
│ 33 ~ 40       │ RX: Sensor Data        │ 0x280~0x287 (Exact Match) │
│               │   IRQ ON → can_receive │ → 센서 데이터 처리          │
│ 41            │ RX: Heartbeat          │ 0x080 (Polling)           │
│ 42            │ RX: Catch-All          │ Mask=0 (모든 ID 수신)      │
│ 43 ~ 64       │ 예비                   │ (미할당)                   │
└───────────────┴────────────────────────┴────────────────────────────┘
```

### DCAN 인터럽트 처리 흐름

```
DCAN1_LINE0_IRQ (IRQ 35, priority 3)
  │
  ├→ process_rx_interrupts(): INT 레지스터에서 pending MsgObj 확인
  │
  ├→ MsgObj 25~32 (Motor FB):
  │     data[0..2] = RPM (i16), data[2..4] = current_mA (u16)
  │     → feedback_queue에 MotorStatus 전송
  │     → feedback_sender → rpmsg → A15 pekko-rust
  │
  ├→ MsgObj 33~40 (Sensor):
  │     → trace 로깅 (향후 처리 확장)
  │
  ├→ MsgObj 41 (Heartbeat):
  │     → 노드 생존 확인
  │
  ├→ MsgObj 42 (Catch-All):
  │     → 미등록 ID 로깅
  │
  └→ Bus-Off 감지 시 → 경고 + soft_reset() 고려
```

### BTR (Bit Timing Register) 설정

```
DCAN FUNCCLK = 20 MHz (DPLL_PER)

1 Mbps: BRP=1, TSEG1=13, TSEG2=4, SJW=3
  tq = (1+1)/20MHz = 100ns
  Bit = (1+13+4) × 100ns = 1.8μs → 실제 1/(1+13+4)×tq

500 kbps: BRP=3, TSEG1=13, TSEG2=4, SJW=3
  tq = (3+1)/20MHz = 200ns
```

---

## TSN (Time-Sensitive Networking)

CoreOS는 AM5729의 CPSW + PRU-ICSS를 활용하여 IEEE TSN 스택을 구현합니다.
이를 통해 로봇 관절 간 결정적(deterministic) Ethernet 통신이 가능합니다.

### TSN 아키텍처 개요

```
┌─────────────────────────────────────────────────────────────────────┐
│                    TSN Network Architecture                          │
│                                                                      │
│  A15 (Linux)                        M4 (RTIC)                       │
│  ┌──────────────┐                   ┌──────────────┐                │
│  │ TSN Actor     │                   │ TSN Tasks     │                │
│  │ ┌──────────┐ │   rpmsg/shared    │ ┌──────────┐ │                │
│  │ │ ptp4l    │ │   memory IPC      │ │ tsn_cycle │ │                │
│  │ │ (gPTP)   │ │ ←───────────────→ │ │ _complete │ │                │
│  │ ├──────────┤ │                   │ ├──────────┤ │                │
│  │ │ phc2sys  │ │                   │ │ tsn_motor │ │                │
│  │ │ (clock)  │ │                   │ │ _sync     │ │                │
│  │ ├──────────┤ │                   │ ├──────────┤ │                │
│  │ │ tc-taprio│ │                   │ │ tsn_rx    │ │                │
│  │ │ (EST)    │ │                   │ │ _receive  │ │                │
│  │ └──────────┘ │                   │ └──────────┘ │                │
│  └──────┬───────┘                   └──────┬───────┘                │
│         │                                  │                         │
│  ┌──────┴──────────────────────────────────┴───────┐                │
│  │              CPSW (0x4848_0000)                   │                │
│  │  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐  │                │
│  │  │ CPTS │ │ ALE  │ │ EST  │ │CPDMA │ │ MDIO │  │                │
│  │  │gPTP  │ │L2 Flt│ │802Qbv│ │DMA Ch│ │PHY   │  │                │
│  │  │HW TS │ │VLAN  │ │ GCL  │ │0-7   │ │Mgmt  │  │                │
│  │  └──────┘ └──────┘ └──────┘ └──────┘ └──────┘  │                │
│  └──────────────────────┬──────────────────────────┘                │
│                         │ RGMII                                      │
│  ┌──────────────────────┴──────────────────────────┐                │
│  │           PRU-ICSS1 (0x4B20_0000)                │                │
│  │  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────────────┐   │                │
│  │  │ IEP  │ │ PRU  │ │MII_RT│ │ Shared Mem   │   │                │
│  │  │200MHz│ │Core  │ │L2 RT │ │ TSN IPC      │   │                │
│  │  │ 5ns  │ │INTC  │ │Mode  │ │ Control+Ring │   │                │
│  │  └──────┘ └──────┘ └──────┘ └──────────────┘   │                │
│  └─────────────────────────────────────────────────┘                │
│                         │                                            │
│                    ┌────┴────┐                                       │
│                    │  PHY    │                                       │
│                    │ 1Gbps  │                                       │
│                    └────┬────┘                                       │
│                         │                                            │
│              ═══════════╧═══════════                                │
│                  TSN Ethernet                                        │
│               (802.1AS + 802.1Qbv)                                  │
└─────────────────────────────────────────────────────────────────────┘
```

### EST (802.1Qbv) Gate Control List — 로봇 제어용

125μs 사이클로 트래픽 클래스별 시간 슬롯을 할당합니다:

```
  0μs          30μs         60μs         100μs      125μs
  │─── TC0 ────│─── TC1 ────│── TC2+TC5 ──│─ TC7+TC6 ─│
  │ Motor Ctrl │ Sensor Data│ Vision+gPTP │ BE+Safety │
  │ (30μs)     │ (30μs)     │ (40μs)      │ (25μs)    │
  │ 최고 우선순위│ 높은 우선순위│ 중간 우선순위 │ 최저 우선순위│

  GCL Entry 0: gate_mask=0x01, interval=30μs  → TC0 (모터 명령)
  GCL Entry 1: gate_mask=0x02, interval=30μs  → TC1 (센서 데이터)
  GCL Entry 2: gate_mask=0x24, interval=40μs  → TC2+TC5 (비전+gPTP)
  GCL Entry 3: gate_mask=0xC0, interval=25μs  → TC7+TC6 (BE+안전)
```

### PRU-ICSS IEP 타이머 TSN 사이클

```
IEP Clock: 200MHz (5ns resolution)
TSN Cycle: 25,000 ticks = 125μs

CMP0 = 25000 (사이클 완료 → Host2 IRQ → tsn_cycle_complete)
CMP1 =  6000 (TC0 종료, 30μs → Motor slot end)
CMP2 = 12000 (TC1 종료, 60μs → Sensor slot end)
CMP3 = 20000 (TC2 종료, 100μs → Vision slot end)

SYNC0: 125μs 주기 펄스 → 외부 장치 동기화 신호
```

### TSN 인터럽트 매핑 (IPU1 Crossbar)

```
PRU-ICSS1 Host 2  →  sys_irq 196  →  NVIC IRQ 40  (TSN 사이클 완료, priority 0x10)
PRU-ICSS1 Host 3  →  sys_irq 197  →  NVIC IRQ 41  (모터 동기 트리거, priority 0x20)
PRU-ICSS1 Host 4  →  sys_irq 198  →  NVIC IRQ 42  (TSN RX 수신, priority 0x50)
```

### CPSW CPTS (IEEE 802.1AS gPTP)

CPTS는 하드웨어 타임스탬프 엔진으로, 네트워크 패킷의 정확한 송수신 시각을 기록합니다:

```
CPTS 모드: 64비트 카운터 + RX 타임스탬프 활성화
이벤트: PUSH → EVENT FIFO (LOW/HIGH/INFO)
gPTP 멀티캐스트: 01:80:C2:00:00:0E → ALE로 Host 포트에 라우팅
EtherType: 0x88F7 (PTP)
```

### 파일 구조

```
cortex-m4/src/pac/
├── cpsw.rs          — CPSW Ethernet Switch TSN 드라이버
│                       CPTS, ALE, EST, CPDMA, MAC, MDIO
├── pru.rs           — PRU-ICSS TSN 드라이버
│                       IEP 타이머, PRU 코어, INTC, MII_RT, 공유 메모리
└── intc.rs          — PRU_ICSS1_HOST2/3/4 인터럽트 추가

cortex-a15/src/actors/
└── tsn.rs           — TSN Actor (linuxptp + tc-taprio 연동)
```
