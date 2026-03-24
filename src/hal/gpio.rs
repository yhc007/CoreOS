//! ─────────────────────────────────────────────────────────────
//! GPIO 드라이버
//! ─────────────────────────────────────────────────────────────

use esp_hal::gpio::{Level, Output, OutputConfig};

/// LED 제어 드라이버
pub struct LedDriver {
    pin: Output<'static>,
}

impl LedDriver {
    /// 새 LED 드라이버 생성 (초기 상태: OFF)
    /// esp-hal 1.0: Output::new(pin, initial_level, config)
    pub fn new(pin: impl esp_hal::gpio::OutputPin + 'static) -> Self {
        let output = Output::new(pin, Level::Low, OutputConfig::default());
        Self { pin: output }
    }

    /// LED 켜기
    pub fn on(&mut self) {
        self.pin.set_high();
    }

    /// LED 끄기
    pub fn off(&mut self) {
        self.pin.set_low();
    }

    /// LED 토글
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }

    /// LED 상태 확인
    pub fn is_on(&self) -> bool {
        self.pin.is_set_high()
    }
}
