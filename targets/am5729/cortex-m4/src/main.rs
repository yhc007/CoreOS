//! ─────────────────────────────────────────────────────────────
//! AM5729 Cortex-M4 IPU1 — RTIC v2 Robot Controller
//! ─────────────────────────────────────────────────────────────
//!
//! AM5729 IPU1 (dual Cortex-M4F) 에서 실행되는 RTIC 기반
//! 하드 리얼타임 로봇 제어 펌웨어.
//!
//! 이 코드는 remoteproc을 통해 A15 Linux에서 로드됩니다:
//!   1. A15 Linux 부팅 → remoteproc 드라이버 초기화
//!   2. /lib/firmware/am5729-ipu1-fw.xem4 로드
//!   3. M4 코어 리셋 해제 → 이 펌웨어 실행
//!   4. rpmsg 채널 생성 → pekko-rust Actor와 IPC
//!
//! Architecture (단일 SoC):
//!   Cortex-A15 (Linux + pekko-rust) ←rpmsg→ Cortex-M4 (이 펌웨어)
//!                                             ↓
//!                                   모터 PWM, IMU I2C, CAN, Encoder
//!
//! ═══════════════════════════════════════════════════════════════
//!  RTIC Dispatchers (AM5729 IPU1 NVIC)
//! ═══════════════════════════════════════════════════════════════
//!
//! RTIC v2는 소프트웨어 태스크 디스패치에 "예비" NVIC IRQ를 사용합니다.
//! AM5729 IPU1의 Crossbar에서 미할당된 IRQ 55~58을 사용:
//!
//!   DISPATCH_SW0 (IRQ 55): priority 1 태스크 (heartbeat)
//!   DISPATCH_SW1 (IRQ 56): priority 2 태스크 (feedback, rpmsg)
//!   DISPATCH_SW2 (IRQ 57): priority 3 태스크 (sensor)
//!   DISPATCH_SW3 (IRQ 58): priority 4 태스크 (motor control)
//!
//! 실제 HW 인터럽트:
//!   MAILBOX5_USER2 (IRQ 26): rpmsg 수신 (binds)

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod pac;
mod ipc;
mod resource_table;

// PID 컨트롤러 (ESP32-S3 프로젝트와 공유 예정)
mod pid;

/// Pre-main AMMU 초기화
///
/// cortex-m-rt가 .data/.bss 초기화보다 먼저 이 함수를 호출합니다.
/// AMMU가 가장 먼저 설정되어야 OCMC RAM 접근도 안전합니다.
///
/// 주의: pre_init에서는 static 변수, 힙, defmt 로깅 모두 사용 불가!
///       순수 레지스터 쓰기만 가능합니다.
#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    pac::ammu::init_ammu();
}

// ═══════════════════════════════════════════════════════════════
// RTIC Application
// ═══════════════════════════════════════════════════════════════
//
// dispatchers: AM5729 IPU1 Crossbar 미할당 IRQ (55~58)
// 이 인터럽트들은 Crossbar에서 RESERVED(0)에 연결되어
// 하드웨어 소스가 없으므로 RTIC가 소프트웨어 pend로만 사용합니다.
#[rtic::app(
    device = cortex_m,
    dispatchers = [
        DISPATCH_SW0,  // IRQ 55 — priority 1 (heartbeat)
        DISPATCH_SW1,  // IRQ 56 — priority 2 (feedback, rpmsg)
        DISPATCH_SW2,  // IRQ 57 — priority 3 (sensor)
        DISPATCH_SW3,  // IRQ 58 — priority 4 (motor control)
    ]
)]
mod app {
    use cortex_m::peripheral::NVIC;
    use defmt::info;
    use heapless::spsc::Queue;
    use rtic_monotonics::systick::prelude::*;

    use crate::pac;
    use crate::pac::ammu;
    use crate::pac::intc;
    use crate::pac::gpio::GpioPort;
    use crate::pac::timer::GpTimer;
    use crate::pac::epwm::Epwm;
    use crate::pac::eqep::Eqep;
    use crate::pac::uart::Uart;
    use crate::pac::dcan::{Dcan, CanFrame, can_id, msg_obj};
    use crate::pac::i2c::{I2c, Imu, ImuData, ImuType};
    use crate::ipc::rpmsg::{
        RpmsgChannel, RpmsgEndpoint,
        RPMSG_ROBOT_CTRL_ADDR, RPMSG_ROBOT_FB_ADDR,
    };
    use crate::pid::PidController;

    // SysTick 모노토닉 타이머 (1MHz)
    rtic_monotonics::systick_monotonic!(Mono, 1_000_000);

    // ── 상수 ──
    /// IPU 클럭 (212 MHz)
    const IPU_CLK_HZ: u32 = 212_000_000;
    /// PWM 서브시스템 클럭 (20 MHz)
    const PWMSS_CLK_HZ: u32 = 20_000_000;
    /// 모터 PWM 주파수 (20 kHz)
    const MOTOR_PWM_FREQ_HZ: u32 = 20_000;
    /// 인코더 분해능 (예: 1024 line encoder → 4096 counts in 4x mode)
    const ENCODER_CPR: u32 = 4096;
    /// 모터/관절 수
    const NUM_JOINTS: usize = 6;

    // ── 공유 리소스 ──
    #[shared]
    struct Shared {
        /// 시스템 상태
        system_state: SystemState,
        /// A15 → M4 명령 큐
        cmd_queue: Queue<RobotCommand, 16>,
        /// M4 → A15 피드백 큐
        feedback_queue: Queue<RobotFeedback, 32>,
        /// rpmsg 채널
        rpmsg: RpmsgChannel,
    }

    // ── 로컬 리소스 (태스크 전용) ──
    #[local]
    struct Local {
        /// LED (GPIO2, Pin 10 — 보드별 조정 필요)
        led: GpioPort,
        /// Heartbeat 카운터
        heartbeat_count: u64,
        /// PID 컨트롤러 배열 (각 관절)
        pid_controllers: [PidController; NUM_JOINTS],
        /// ePWM 드라이버 (모터 출력)
        motor_pwms: [Epwm; 3],  // ePWM1~3 (6축 = 3개 ePWM × 2ch)
        /// eQEP 인코더 (위치 피드백)
        encoders: [Eqep; 3],    // eQEP1~3
        /// 1kHz 제어 루프 카운터
        control_tick: u64,
        /// I2C1 드라이버
        i2c1: I2c,
        /// IMU 센서 타입 (감지 결과)
        imu_type: ImuType,
        /// IMU I2C 주소
        imu_addr: u8,
        /// DCAN1 드라이버
        dcan1: Dcan,
        /// CAN 수신 프레임 버퍼
        can_rx_buf: [CanFrame; 8],
    }

    // ── 시스템 상태 ──
    #[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
    pub enum SystemState {
        Initializing,
        Ready,
        Running,
        Warning,
        EmergencyStop,
    }

    // ── 로봇 명령 (A15 → M4, rpmsg 경유) ──
    #[derive(Clone, Copy, Debug, defmt::Format)]
    pub enum RobotCommand {
        SetMotorSpeed { motor_id: u8, rpm: i16 },
        SetServoAngle { servo_id: u8, angle_deg: i16 },
        SetPidGains { joint: u8, kp: f32, ki: f32, kd: f32 },
        EmergencyStop,
        ClearEstop,
        RequestStatus,
        Ping,
    }

    // ── 로봇 피드백 (M4 → A15, rpmsg 경유) ──
    #[derive(Clone, Copy, Debug, defmt::Format)]
    pub enum RobotFeedback {
        MotorStatus { motor_id: u8, actual_rpm: i16, current_ma: u16 },
        EncoderData { joint_id: u8, position: u32, speed_rpm: i16 },
        ImuData { accel: [i16; 3], gyro: [i16; 3] },
        SystemStatus { state: SystemState, tick: u64 },
        Pong,
    }

    // ────────────────────────────────────────────────
    // 초기화
    // ────────────────────────────────────────────────
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("══════════════════════════════════════");
        info!("  AM5729 Cortex-M4 IPU1 — RTIC v2");
        info!("  Robot Controller Firmware v0.2.0");
        info!("══════════════════════════════════════");

        // ══════════════════════════════════════════════
        // 0. AMMU 초기화 (pre_init에서 1차 완료, 여기서 재확인)
        // ══════════════════════════════════════════════
        ammu::init_ammu();
        info!("[INIT] AMMU configured — peripheral access enabled");

        let ammu_ok = ammu::verify_ammu_mappings();
        info!("[INIT] AMMU verify: {}/5 peripherals accessible", ammu_ok);
        if ammu_ok == 0 {
            defmt::error!("[INIT] AMMU FAILED — no peripherals reachable!");
        }

        // ══════════════════════════════════════════════
        // 1. Crossbar + NVIC 인터럽트 초기화
        // ══════════════════════════════════════════════
        // Crossbar에 HW 인터럽트 소스 매핑 + dispatcher IRQ 예약
        intc::init_crossbar_and_nvic();
        info!("[INIT] Crossbar + NVIC configured");

        // Crossbar 매핑 검증
        let crossbar_map = intc::verify_crossbar();
        for (irq, sys_irq) in crossbar_map.iter() {
            info!("  IRQ {} → sys_irq {}", irq, sys_irq);
        }

        // SysTick 모노토닉 초기화 (212MHz IPU 클럭 기준)
        Mono::start(cx.core.SYST, IPU_CLK_HZ);

        // ── 페리페럴 초기화 (AMMU 설정 완료 후에만 가능!) ──

        // 2. GPIO (LED + E-Stop 입력)
        let mut led = unsafe { GpioPort::new(pac::addr::GPIO2) };
        led.set_output(10); // GPIO2_10 = LED
        led.set_low(10);
        info!("[INIT] GPIO2: LED on pin 10");

        // 3. ePWM (모터 제어 — 20kHz center-aligned)
        let motor_pwms = unsafe {[
            Epwm::new(pac::addr::EPWM1),
            Epwm::new(pac::addr::EPWM2),
            Epwm::new(pac::addr::EPWM3),
        ]};
        for pwm in motor_pwms.iter() {
            pwm.init_motor_pwm(MOTOR_PWM_FREQ_HZ, PWMSS_CLK_HZ);
        }
        info!("[INIT] ePWM1-3: Motor PWM @ {}kHz", MOTOR_PWM_FREQ_HZ / 1000);

        // 4. eQEP (인코더)
        let encoders = unsafe {[
            Eqep::new(pac::addr::EQEP1),
            Eqep::new(pac::addr::EQEP2),
            Eqep::new(pac::addr::EQEP3),
        ]};
        for enc in encoders.iter() {
            enc.init(ENCODER_CPR, PWMSS_CLK_HZ);
        }
        info!("[INIT] eQEP1-3: Encoders @ {} CPR", ENCODER_CPR);

        // 5. UART3 (디버그 콘솔)
        let uart = unsafe { Uart::new(pac::addr::UART3) };
        uart.init_921600();
        info!("[INIT] UART3: Debug console @ 921600 baud");

        // 6. DCAN1 (외부 장치 통신)
        let dcan1 = unsafe { Dcan::new(pac::addr::DCAN1) };
        dcan1.init_1mbps();
        info!("[INIT] DCAN1: CAN bus @ 1Mbps");

        // DCAN1 수신 Message Object 일괄 설정
        //   MsgObj 25~32: 모터 피드백 (ID 0x180~0x187, IRQ ON)
        //   MsgObj 33~40: 센서 데이터 (ID 0x280~0x287, IRQ ON)
        //   MsgObj 41:    하트비트    (ID 0x080, 폴링)
        //   MsgObj 42:    Catch-all   (모든 ID, 디버그)
        dcan1.setup_robot_rx_objects();
        info!("[INIT] DCAN1: RX message objects configured (25~42)");

        // 7. I2C1 + IMU 초기화
        let i2c1 = unsafe { I2c::new(pac::addr::I2C1) };
        i2c1.init_400khz();
        let i2c_rev = i2c1.read_revision();
        info!("[INIT] I2C1: Fast Mode 400kHz, rev=0x{:08X}", i2c_rev);

        // IMU 자동 감지 (MPU6050 / ICM42688 / BNO055)
        let (imu_type, imu_addr) = if let Some(imu) = Imu::new(&i2c1) {
            let imu_type = imu.sensor_type();
            let imu_addr = imu.address();
            info!("[INIT] IMU detected: {:?} @ 0x{:02X}", imu_type, imu_addr);

            // IMU 초기화 (센서별 레지스터 설정)
            match imu.init() {
                Ok(()) => info!("[INIT] IMU initialized successfully"),
                Err(e) => defmt::error!("[INIT] IMU init failed: {:?}", e),
            }
            (imu_type, imu_addr)
        } else {
            defmt::warn!("[INIT] No IMU detected on I2C1 (0x68/0x69)");
            (ImuType::Unknown(0), 0x68)
        };

        // 8. PID 컨트롤러 (6축)
        let pid_controllers = [
            PidController::new(2.0, 0.5, 0.1, -1000.0, 1000.0),
            PidController::new(2.0, 0.5, 0.1, -1000.0, 1000.0),
            PidController::new(2.0, 0.5, 0.1, -1000.0, 1000.0),
            PidController::new(1.5, 0.3, 0.05, -500.0, 500.0),
            PidController::new(1.5, 0.3, 0.05, -500.0, 500.0),
            PidController::new(1.0, 0.2, 0.02, -200.0, 200.0),
        ];
        info!("[INIT] PID controllers for {} joints", NUM_JOINTS);

        // 9. rpmsg 채널 (A15 ↔ M4 IPC)
        let mut rpmsg = RpmsgChannel::new(
            0x9F00_0000,  // RX vring (Linux가 실제 할당)
            0x9F04_0000,  // TX vring
        );
        rpmsg.init();
        info!("[INIT] rpmsg channel initialized, waiting for A15...");

        // rpmsg 엔드포인트 등록 (Name Service Announcement)
        let ctrl_ep = RpmsgEndpoint::new("rpmsg-robot-ctrl", RPMSG_ROBOT_CTRL_ADDR);
        let fb_ep = RpmsgEndpoint::new("rpmsg-robot-fb", RPMSG_ROBOT_FB_ADDR);
        rpmsg.announce_endpoint(&ctrl_ep);
        rpmsg.announce_endpoint(&fb_ep);
        info!("[INIT] rpmsg endpoints announced: ctrl={}, fb={}",
              RPMSG_ROBOT_CTRL_ADDR, RPMSG_ROBOT_FB_ADDR);

        // ── 주기적 태스크 시작 ──
        heartbeat::spawn().ok();
        motor_control_loop::spawn().ok();
        sensor_read_loop::spawn().ok();
        feedback_sender::spawn().ok();

        info!("[INIT] All tasks spawned. System ready.");

        (
            Shared {
                system_state: SystemState::Ready,
                cmd_queue: Queue::new(),
                feedback_queue: Queue::new(),
                rpmsg,
            },
            Local {
                led,
                heartbeat_count: 0,
                pid_controllers,
                motor_pwms,
                encoders,
                control_tick: 0,
                i2c1,
                imu_type,
                imu_addr,
                dcan1,
                can_rx_buf: [CanFrame::empty(); 8],
            },
        )
    }

    // ────────────────────────────────────────────────
    // Idle (저전력 대기)
    // ────────────────────────────────────────────────
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    // ────────────────────────────────────────────────
    // 태스크: Heartbeat (최저 우선순위)
    //   500ms 주기 — LED 점멸 + 상태 보고
    // ────────────────────────────────────────────────
    #[task(local = [led, heartbeat_count], shared = [system_state, feedback_queue], priority = 1)]
    async fn heartbeat(mut cx: heartbeat::Context) {
        loop {
            *cx.local.heartbeat_count += 1;
            let count = *cx.local.heartbeat_count;

            let state = cx.shared.system_state.lock(|s| *s);

            // 상태별 LED 패턴
            match state {
                SystemState::Ready => {
                    cx.local.led.toggle(10);
                }
                SystemState::Running => {
                    cx.local.led.set_high(10);
                }
                SystemState::EmergencyStop => {
                    cx.local.led.toggle(10);
                }
                _ => {
                    cx.local.led.toggle(10);
                }
            }

            // 10초마다 상태 보고
            if count % 20 == 0 {
                info!("[HEARTBEAT] #{} state={:?}", count, state);
                cx.shared.feedback_queue.lock(|q| {
                    q.enqueue(RobotFeedback::SystemStatus {
                        state,
                        tick: count,
                    }).ok();
                });
            }

            Mono::delay(500.millis()).await;
        }
    }

    // ────────────────────────────────────────────────
    // 태스크: 모터 제어 루프 (최고 우선순위)
    //   1kHz (1ms 주기) — 하드 리얼타임
    // ────────────────────────────────────────────────
    #[task(
        local = [pid_controllers, motor_pwms, encoders, control_tick],
        shared = [cmd_queue, system_state, feedback_queue],
        priority = 4
    )]
    async fn motor_control_loop(mut cx: motor_control_loop::Context) {
        info!("[MOTOR] Control loop started — 1kHz");

        let mut setpoints: [f32; NUM_JOINTS] = [0.0; NUM_JOINTS];
        const DT: f32 = 0.001; // 1ms

        loop {
            *cx.local.control_tick += 1;
            let state = cx.shared.system_state.lock(|s| *s);

            match state {
                SystemState::EmergencyStop => {
                    for pwm in cx.local.motor_pwms.iter() {
                        pwm.emergency_stop();
                    }
                }
                SystemState::Running => {
                    // cmd_queue에서 새 명령 확인
                    if let Some(cmd) = cx.shared.cmd_queue.lock(|q| q.dequeue()) {
                        match cmd {
                            RobotCommand::SetMotorSpeed { motor_id, rpm } => {
                                if (motor_id as usize) < NUM_JOINTS {
                                    setpoints[motor_id as usize] = rpm as f32;
                                }
                            }
                            RobotCommand::SetPidGains { joint, kp, ki, kd } => {
                                if (joint as usize) < NUM_JOINTS {
                                    cx.local.pid_controllers[joint as usize] =
                                        PidController::new(kp, ki, kd, -1000.0, 1000.0);
                                }
                            }
                            RobotCommand::EmergencyStop => {
                                cx.shared.system_state.lock(|s| *s = SystemState::EmergencyStop);
                                for pwm in cx.local.motor_pwms.iter() {
                                    pwm.emergency_stop();
                                }
                                continue;
                            }
                            _ => {}
                        }
                    }

                    // 각 관절 PID 업데이트
                    for i in 0..3 {
                        let actual_rpm = cx.local.encoders[i].read_speed_rpm(ENCODER_CPR);
                        let output = cx.local.pid_controllers[i].update(
                            setpoints[i], actual_rpm, DT
                        );
                        let duty = if output >= 0.0 {
                            output.min(1000.0) as u16
                        } else {
                            (-output).min(1000.0) as u16
                        };
                        cx.local.motor_pwms[i].set_duty_permille(duty);
                    }

                    // 50ms마다 인코더 상태 피드백
                    if *cx.local.control_tick % 50 == 0 {
                        for i in 0..3 {
                            let pos = cx.local.encoders[i].read_position();
                            let rpm = cx.local.encoders[i].read_speed_rpm(ENCODER_CPR) as i16;
                            cx.shared.feedback_queue.lock(|q| {
                                q.enqueue(RobotFeedback::EncoderData {
                                    joint_id: i as u8,
                                    position: pos,
                                    speed_rpm: rpm,
                                }).ok();
                            });
                        }
                    }
                }
                _ => {
                    // Ready/Initializing — 모터 대기
                }
            }

            Mono::delay(1.millis()).await;
        }
    }

    // ────────────────────────────────────────────────
    // 태스크: 센서 읽기 루프 (높은 우선순위)
    //   1kHz IMU 읽기 (I2C1 → MPU6050/ICM42688)
    // ────────────────────────────────────────────────
    #[task(local = [i2c1, imu_type, imu_addr], shared = [feedback_queue], priority = 3)]
    async fn sensor_read_loop(mut cx: sensor_read_loop::Context) {
        info!("[SENSOR] Read loop started — 1kHz");
        info!("[SENSOR] IMU: {:?} @ 0x{:02X}", cx.local.imu_type, cx.local.imu_addr);

        let mut imu_tick: u64 = 0;
        let addr = *cx.local.imu_addr;
        let imu_type = *cx.local.imu_type;

        // IMU 데이터 레지스터 시작 주소 (센서별)
        let (data_reg, data_len) = match imu_type {
            ImuType::Mpu6050 => (0x3Bu8, 14usize),    // ACCEL_XOUT_H → 14 bytes
            ImuType::Icm42688 => (0x1Du8, 14usize),   // TEMP_DATA1 → 14 bytes
            _ => (0x3Bu8, 14usize),                     // 기본값
        };

        loop {
            imu_tick += 1;

            // I2C burst read로 IMU 6축 데이터 읽기
            let mut raw = [0u8; 14];
            let imu_ok = cx.local.i2c1.read_reg(addr, data_reg, &mut raw[..data_len]).is_ok();

            if imu_ok {
                // 센서별 바이트 순서 파싱
                let (accel, gyro) = match imu_type {
                    ImuType::Mpu6050 => {
                        // MPU6050: [Ax_H, Ax_L, Ay_H, Ay_L, Az_H, Az_L,
                        //           Temp_H, Temp_L,
                        //           Gx_H, Gx_L, Gy_H, Gy_L, Gz_H, Gz_L]
                        let accel = [
                            i16::from_be_bytes([raw[0], raw[1]]),
                            i16::from_be_bytes([raw[2], raw[3]]),
                            i16::from_be_bytes([raw[4], raw[5]]),
                        ];
                        let gyro = [
                            i16::from_be_bytes([raw[8], raw[9]]),
                            i16::from_be_bytes([raw[10], raw[11]]),
                            i16::from_be_bytes([raw[12], raw[13]]),
                        ];
                        (accel, gyro)
                    }
                    ImuType::Icm42688 => {
                        // ICM42688: [Temp_H, Temp_L,
                        //            Ax_H, Ax_L, Ay_H, Ay_L, Az_H, Az_L,
                        //            Gx_H, Gx_L, Gy_H, Gy_L, Gz_H, Gz_L]
                        let accel = [
                            i16::from_be_bytes([raw[2], raw[3]]),
                            i16::from_be_bytes([raw[4], raw[5]]),
                            i16::from_be_bytes([raw[6], raw[7]]),
                        ];
                        let gyro = [
                            i16::from_be_bytes([raw[8], raw[9]]),
                            i16::from_be_bytes([raw[10], raw[11]]),
                            i16::from_be_bytes([raw[12], raw[13]]),
                        ];
                        (accel, gyro)
                    }
                    _ => {
                        ([0i16; 3], [0i16; 3])
                    }
                };

                // 10ms마다 (100Hz) 피드백 전송
                if imu_tick % 10 == 0 {
                    cx.shared.feedback_queue.lock(|q| {
                        q.enqueue(RobotFeedback::ImuData { accel, gyro }).ok();
                    });
                }
            } else if imu_tick % 1000 == 0 {
                // I2C 에러 시 1초마다 경고
                defmt::warn!("[SENSOR] IMU read failed at tick {}", imu_tick);
            }

            Mono::delay(1.millis()).await;
        }
    }

    // ────────────────────────────────────────────────
    // 태스크: 피드백 전송 (rpmsg → A15)
    //   10ms 주기 — 피드백 큐를 A15로 전송
    // ────────────────────────────────────────────────
    #[task(shared = [feedback_queue, rpmsg], priority = 2)]
    async fn feedback_sender(mut cx: feedback_sender::Context) {
        info!("[FEEDBACK] Sender started — 10ms cycle");

        loop {
            let msg = cx.shared.feedback_queue.lock(|q| q.dequeue());

            if let Some(fb) = msg {
                let mut buf = [0u8; 64];
                let len = encode_feedback(&fb, &mut buf);

                cx.shared.rpmsg.lock(|rpmsg| {
                    rpmsg.send(
                        crate::ipc::rpmsg::RPMSG_ROBOT_FB_ADDR,
                        crate::ipc::rpmsg::RPMSG_ROBOT_CTRL_ADDR,
                        &buf[..len],
                    );
                });
            }

            Mono::delay(10.millis()).await;
        }
    }

    // ────────────────────────────────────────────────
    // 인터럽트: rpmsg 수신 (Mailbox5 User2 IRQ)
    //   A15에서 명령이 올 때마다 트리거
    // ────────────────────────────────────────────────
    //
    // `binds = MAILBOX5_USER2`: NVIC IRQ 26 (Crossbar → Mailbox5 User2)
    // 이전의 SWI3 대신 실제 HW 인터럽트 사용
    #[task(binds = MAILBOX5_USER2, shared = [cmd_queue, system_state, rpmsg], priority = 2)]
    fn rpmsg_receive(mut cx: rpmsg_receive::Context) {
        // Mailbox IRQ → rpmsg 메시지 수신
        let result = cx.shared.rpmsg.lock(|rpmsg| {
            rpmsg.clear_irq();
            rpmsg.recv()
        });

        if let Some((_src, _dst, payload_addr, len)) = result {
            let mut buf = [0u8; 64];
            let copied = RpmsgChannel::copy_payload(payload_addr, &mut buf, len);

            if let Some(cmd) = decode_command(&buf[..copied]) {
                match cmd {
                    RobotCommand::EmergencyStop => {
                        cx.shared.system_state.lock(|s| *s = SystemState::EmergencyStop);
                        info!("[RPMSG] EMERGENCY STOP from A15!");
                    }
                    RobotCommand::ClearEstop => {
                        cx.shared.system_state.lock(|s| *s = SystemState::Ready);
                        info!("[RPMSG] E-Stop cleared, state → Ready");
                    }
                    _ => {
                        cx.shared.cmd_queue.lock(|q| {
                            q.enqueue(cmd).ok();
                        });
                    }
                }
            }
        }
    }

    // ────────────────────────────────────────────────
    // 인터럽트: DCAN1 수신 (DCAN1 Line 0 IRQ)
    //   CAN 버스에서 모터 피드백/센서 데이터 수신
    // ────────────────────────────────────────────────
    //
    // `binds = DCAN1_LINE0`: NVIC IRQ 35 (Crossbar → DCAN1 Line 0)
    // CAN 프레임 수신 시 인터럽트 발생 → MsgObj 읽기 → 처리
    #[task(
        binds = DCAN1_LINE0,
        local = [dcan1, can_rx_buf],
        shared = [feedback_queue, system_state],
        priority = 3
    )]
    fn can_receive(mut cx: can_receive::Context) {
        // pending된 모든 RX 프레임 읽기
        let count = cx.local.dcan1.process_rx_interrupts(cx.local.can_rx_buf);

        for i in 0..count {
            let frame = &cx.local.can_rx_buf[i];

            // Message Lost 경고
            if frame.msg_lost {
                defmt::warn!("[CAN] MsgObj {} overrun! ID=0x{:03X}", frame.msg_obj, frame.id);
            }

            // MsgObj 번호별 처리 분기
            match frame.msg_obj {
                // ── 모터 피드백 (MsgObj 25~32, ID 0x180~0x187) ──
                // 데이터 형식: [RPM_H, RPM_L, Current_H, Current_L, Status, 0, 0, 0]
                obj @ 25..=32 => {
                    let motor_id = obj - msg_obj::RX_MOTOR_FB_BASE;
                    if frame.dlc >= 4 {
                        let actual_rpm = i16::from_be_bytes([frame.data[0], frame.data[1]]);
                        let current_ma = u16::from_be_bytes([frame.data[2], frame.data[3]]);

                        cx.shared.feedback_queue.lock(|q| {
                            q.enqueue(RobotFeedback::MotorStatus {
                                motor_id,
                                actual_rpm,
                                current_ma,
                            }).ok();
                        });
                    }
                }

                // ── 센서 데이터 (MsgObj 33~40, ID 0x280~0x287) ──
                // 형식은 센서 타입에 따라 다름 — 일단 raw로 피드백
                obj @ 33..=40 => {
                    let _sensor_id = obj - msg_obj::RX_SENSOR_BASE;
                    // 센서 데이터는 A15에 raw로 전달
                    // 추후 센서별 파싱 추가
                    defmt::trace!("[CAN] Sensor {} data: {} bytes", _sensor_id, frame.dlc);
                }

                // ── 하트비트 (MsgObj 41, ID 0x080) ──
                41 => {
                    // 모터 드라이버 생존 확인
                    defmt::trace!("[CAN] Heartbeat from motor driver");
                }

                // ── Catch-all (MsgObj 42) ──
                42 => {
                    defmt::trace!("[CAN] Unknown ID=0x{:03X}, DLC={}", frame.id, frame.dlc);
                }

                _ => {}
            }
        }

        // Bus-Off 복구 체크
        if cx.local.dcan1.is_bus_off() {
            defmt::error!("[CAN] Bus-Off detected!");
            cx.shared.system_state.lock(|s| {
                if *s == SystemState::Running {
                    *s = SystemState::Warning;
                }
            });
        }
    }

    // ────────────────────────────────────────────────
    // 헬퍼: 피드백 인코딩 (간단한 바이너리)
    // ────────────────────────────────────────────────
    fn encode_feedback(fb: &RobotFeedback, buf: &mut [u8]) -> usize {
        match fb {
            RobotFeedback::EncoderData { joint_id, position, speed_rpm } => {
                buf[0] = 0x01;
                buf[1] = *joint_id;
                buf[2..6].copy_from_slice(&position.to_le_bytes());
                buf[6..8].copy_from_slice(&speed_rpm.to_le_bytes());
                8
            }
            RobotFeedback::ImuData { accel, gyro } => {
                buf[0] = 0x02;
                for (i, &a) in accel.iter().enumerate() {
                    buf[1 + i * 2..3 + i * 2].copy_from_slice(&a.to_le_bytes());
                }
                for (i, &g) in gyro.iter().enumerate() {
                    buf[7 + i * 2..9 + i * 2].copy_from_slice(&g.to_le_bytes());
                }
                13
            }
            RobotFeedback::SystemStatus { state, tick } => {
                buf[0] = 0x03;
                buf[1] = *state as u8;
                buf[2..10].copy_from_slice(&tick.to_le_bytes());
                10
            }
            RobotFeedback::Pong => {
                buf[0] = 0xFF;
                1
            }
            RobotFeedback::MotorStatus { motor_id, actual_rpm, current_ma } => {
                buf[0] = 0x04;
                buf[1] = *motor_id;
                buf[2..4].copy_from_slice(&actual_rpm.to_le_bytes());
                buf[4..6].copy_from_slice(&current_ma.to_le_bytes());
                6
            }
        }
    }

    // ────────────────────────────────────────────────
    // 헬퍼: 명령 디코딩
    // ────────────────────────────────────────────────
    fn decode_command(data: &[u8]) -> Option<RobotCommand> {
        if data.is_empty() {
            return None;
        }
        match data[0] {
            0x01 => {
                if data.len() < 4 { return None; }
                let motor_id = data[1];
                let rpm = i16::from_le_bytes([data[2], data[3]]);
                Some(RobotCommand::SetMotorSpeed { motor_id, rpm })
            }
            0x02 => {
                if data.len() < 4 { return None; }
                let servo_id = data[1];
                let angle_deg = i16::from_le_bytes([data[2], data[3]]);
                Some(RobotCommand::SetServoAngle { servo_id, angle_deg })
            }
            0x03 => {
                if data.len() < 14 { return None; }
                let joint = data[1];
                let kp = f32::from_le_bytes([data[2], data[3], data[4], data[5]]);
                let ki = f32::from_le_bytes([data[6], data[7], data[8], data[9]]);
                let kd = f32::from_le_bytes([data[10], data[11], data[12], data[13]]);
                Some(RobotCommand::SetPidGains { joint, kp, ki, kd })
            }
            0xE0 => Some(RobotCommand::EmergencyStop),
            0xE1 => Some(RobotCommand::ClearEstop),
            0xF0 => Some(RobotCommand::RequestStatus),
            0xFF => Some(RobotCommand::Ping),
            _ => None,
        }
    }
}
