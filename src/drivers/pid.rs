//! ─────────────────────────────────────────────────────────────
//! PID 컨트롤러 — no_std, no_alloc
//! ─────────────────────────────────────────────────────────────
//!
//! 로봇 제어의 핵심 — 모터 속도/위치 제어에 사용됩니다.
//! Anti-windup, 출력 클램핑, derivative kick 방지 포함.
//!
//! 사용 예:
//! ```
//! let mut pid = PidController::new(1.0, 0.1, 0.05);
//! pid.set_limits(-1.0, 1.0);
//! pid.set_setpoint(100.0); // 목표 RPM
//!
//! // 제어 루프 (1kHz)
//! let output = pid.update(current_rpm, 0.001); // dt = 1ms
//! motor.set_duty(output);
//! ```

/// PID 컨트롤러 파라미터
#[derive(Clone, Copy, Debug)]
pub struct PidGains {
    pub kp: f32, // 비례 게인
    pub ki: f32, // 적분 게인
    pub kd: f32, // 미분 게인
}

/// PID 컨트롤러 (no_std 호환)
pub struct PidController {
    gains: PidGains,
    setpoint: f32,

    // 내부 상태
    integral: f32,
    prev_measurement: f32,
    prev_error: f32,

    // 출력 제한
    output_min: f32,
    output_max: f32,

    // Anti-windup 적분 제한
    integral_min: f32,
    integral_max: f32,

    // 초기화 플래그
    initialized: bool,
}

impl PidController {
    /// 새 PID 컨트롤러 생성
    pub const fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            gains: PidGains { kp, ki, kd },
            setpoint: 0.0,
            integral: 0.0,
            prev_measurement: 0.0,
            prev_error: 0.0,
            output_min: f32::MIN,
            output_max: f32::MAX,
            integral_min: -1000.0,
            integral_max: 1000.0,
            initialized: false,
        }
    }

    /// 출력 범위 설정
    pub fn set_limits(&mut self, min: f32, max: f32) {
        self.output_min = min;
        self.output_max = max;
    }

    /// 적분 제한 설정 (Anti-windup)
    pub fn set_integral_limits(&mut self, min: f32, max: f32) {
        self.integral_min = min;
        self.integral_max = max;
    }

    /// 목표값 설정
    pub fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
    }

    /// PID 게인 동적 업데이트 (튜닝용)
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.gains = PidGains { kp, ki, kd };
    }

    /// PID 업데이트 — 매 제어 주기마다 호출
    ///
    /// - `measurement`: 현재 측정값 (예: 실제 RPM)
    /// - `dt`: 시간 간격 (초)
    /// - returns: 제어 출력
    pub fn update(&mut self, measurement: f32, dt: f32) -> f32 {
        let error = self.setpoint - measurement;

        // ── Proportional ──
        let p_term = self.gains.kp * error;

        // ── Integral (with anti-windup clamping) ──
        self.integral += error * dt;
        self.integral = clamp(self.integral, self.integral_min, self.integral_max);
        let i_term = self.gains.ki * self.integral;

        // ── Derivative (on measurement to avoid derivative kick) ──
        let d_term = if self.initialized && dt > 0.0 {
            // 미분을 measurement 변화량으로 계산 (setpoint 변경 시 spike 방지)
            let d_measurement = (measurement - self.prev_measurement) / dt;
            -self.gains.kd * d_measurement
        } else {
            0.0
        };

        // ── 합산 및 클램핑 ──
        let output = clamp(p_term + i_term + d_term, self.output_min, self.output_max);

        // ── 상태 저장 ──
        self.prev_error = error;
        self.prev_measurement = measurement;
        self.initialized = true;

        output
    }

    /// 컨트롤러 상태 초기화
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = 0.0;
        self.prev_error = 0.0;
        self.initialized = false;
    }

    /// 현재 에러값 반환
    pub fn error(&self) -> f32 {
        self.prev_error
    }

    /// 현재 적분값 반환 (디버깅용)
    pub fn integral_value(&self) -> f32 {
        self.integral
    }
}

/// no_std용 clamp 함수
#[inline]
fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

// ── 테스트 ──
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_proportional() {
        let mut pid = PidController::new(1.0, 0.0, 0.0);
        pid.set_setpoint(100.0);
        let output = pid.update(80.0, 0.001);
        assert!((output - 20.0).abs() < 0.001);
    }

    #[test]
    fn test_pid_integral_accumulation() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        pid.set_setpoint(100.0);

        // 10번 업데이트, dt=0.1, error=20 → integral = 20*0.1*10 = 20
        for _ in 0..10 {
            pid.update(80.0, 0.1);
        }
        assert!((pid.integral_value() - 20.0).abs() < 0.001);
    }

    #[test]
    fn test_pid_output_clamping() {
        let mut pid = PidController::new(10.0, 0.0, 0.0);
        pid.set_limits(-1.0, 1.0);
        pid.set_setpoint(100.0);
        let output = pid.update(0.0, 0.001);
        assert!((output - 1.0).abs() < 0.001); // clamped to max
    }

    #[test]
    fn test_pid_anti_windup() {
        let mut pid = PidController::new(0.0, 100.0, 0.0);
        pid.set_integral_limits(-10.0, 10.0);
        pid.set_setpoint(100.0);

        for _ in 0..1000 {
            pid.update(0.0, 0.01);
        }
        assert!(pid.integral_value() <= 10.0);
    }

    #[test]
    fn test_pid_reset() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        pid.set_setpoint(100.0);
        pid.update(50.0, 0.01);
        pid.reset();
        assert!((pid.integral_value() - 0.0).abs() < 0.001);
        assert!((pid.error() - 0.0).abs() < 0.001);
    }
}
