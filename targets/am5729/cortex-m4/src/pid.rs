//! ─────────────────────────────────────────────────────────────
//! PID 컨트롤러 — no_std, no_alloc (AM5729 M4용)
//! ─────────────────────────────────────────────────────────────
//!
//! ESP32-S3 프로젝트와 동일한 PID 알고리즘.
//! 추후 workspace crate로 통합 예정.

/// PID 컨트롤러 (no_std 호환)
#[derive(Clone, Copy)]
pub struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    prev_measurement: f32,
    output_min: f32,
    output_max: f32,
    integral_min: f32,
    integral_max: f32,
    initialized: bool,
}

impl PidController {
    /// 새 PID 컨트롤러 (출력 범위 포함)
    pub const fn new(kp: f32, ki: f32, kd: f32, output_min: f32, output_max: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_measurement: 0.0,
            output_min,
            output_max,
            integral_min: output_min,
            integral_max: output_max,
            initialized: false,
        }
    }

    /// PID 업데이트
    /// - `setpoint`: 목표값
    /// - `measurement`: 현재 측정값
    /// - `dt`: 시간 간격 (초)
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        // Proportional
        let p_term = self.kp * error;

        // Integral (anti-windup)
        self.integral += error * dt;
        self.integral = clamp(self.integral, self.integral_min, self.integral_max);
        let i_term = self.ki * self.integral;

        // Derivative (on measurement → no derivative kick)
        let d_term = if self.initialized && dt > 0.0 {
            let d_meas = (measurement - self.prev_measurement) / dt;
            -self.kd * d_meas
        } else {
            0.0
        };

        let output = clamp(p_term + i_term + d_term, self.output_min, self.output_max);

        self.prev_measurement = measurement;
        self.initialized = true;

        output
    }

    /// 상태 초기화
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = 0.0;
        self.initialized = false;
    }
}

#[inline]
fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min { min }
    else if value > max { max }
    else { value }
}
