//! ─────────────────────────────────────────────────────────────
//! AM5729 GPIO 레지스터 정의
//! ─────────────────────────────────────────────────────────────
//! TRM SPRUHZ6 — Chapter 26: General-Purpose Interface (GPIO)
//!
//! 각 GPIO 모듈은 32핀을 제어하며 총 8개 모듈 (GPIO1~GPIO8)

/// GPIO 레지스터 오프셋 (모듈 베이스 기준)
pub mod offset {
    /// GPIO Revision (RO)
    pub const GPIO_REVISION: u32 = 0x000;
    /// GPIO System Configuration
    pub const GPIO_SYSCONFIG: u32 = 0x010;
    /// GPIO End-of-Interrupt
    pub const GPIO_EOI: u32 = 0x020;
    /// GPIO IRQ Status Raw (0/1)
    pub const GPIO_IRQSTATUS_RAW_0: u32 = 0x024;
    pub const GPIO_IRQSTATUS_RAW_1: u32 = 0x028;
    /// GPIO IRQ Status (0/1)
    pub const GPIO_IRQSTATUS_0: u32 = 0x02C;
    pub const GPIO_IRQSTATUS_1: u32 = 0x030;
    /// GPIO IRQ Enable Set (0/1)
    pub const GPIO_IRQSTATUS_SET_0: u32 = 0x034;
    pub const GPIO_IRQSTATUS_SET_1: u32 = 0x038;
    /// GPIO IRQ Enable Clear (0/1)
    pub const GPIO_IRQSTATUS_CLR_0: u32 = 0x03C;
    pub const GPIO_IRQSTATUS_CLR_1: u32 = 0x040;
    /// GPIO Output Enable (0=output, 1=input)
    pub const GPIO_OE: u32 = 0x134;
    /// GPIO Data In (읽기)
    pub const GPIO_DATAIN: u32 = 0x138;
    /// GPIO Data Out (쓰기)
    pub const GPIO_DATAOUT: u32 = 0x13C;
    /// GPIO Level Detect Low
    pub const GPIO_LEVELDETECT0: u32 = 0x140;
    /// GPIO Level Detect High
    pub const GPIO_LEVELDETECT1: u32 = 0x144;
    /// GPIO Rising Edge Detect
    pub const GPIO_RISINGDETECT: u32 = 0x148;
    /// GPIO Falling Edge Detect
    pub const GPIO_FALLINGDETECT: u32 = 0x14C;
    /// GPIO Set Data Out (비트별 세트)
    pub const GPIO_SETDATAOUT: u32 = 0x194;
    /// GPIO Clear Data Out (비트별 클리어)
    pub const GPIO_CLEARDATAOUT: u32 = 0x190;
}

/// GPIO 포트 드라이버
pub struct GpioPort {
    base: u32,
}

impl GpioPort {
    /// 새 GPIO 포트 생성
    ///
    /// # Safety
    /// base 주소가 올바른 GPIO 모듈 주소여야 합니다.
    pub const unsafe fn new(base: u32) -> Self {
        Self { base }
    }

    /// 핀을 출력으로 설정
    pub fn set_output(&self, pin: u8) {
        assert!(pin < 32);
        unsafe {
            super::clear_bits(self.base + offset::GPIO_OE, 1 << pin);
        }
    }

    /// 핀을 입력으로 설정
    pub fn set_input(&self, pin: u8) {
        assert!(pin < 32);
        unsafe {
            super::set_bits(self.base + offset::GPIO_OE, 1 << pin);
        }
    }

    /// 핀 HIGH 출력
    pub fn set_high(&self, pin: u8) {
        assert!(pin < 32);
        unsafe {
            super::write_reg(self.base + offset::GPIO_SETDATAOUT, 1 << pin);
        }
    }

    /// 핀 LOW 출력
    pub fn set_low(&self, pin: u8) {
        assert!(pin < 32);
        unsafe {
            super::write_reg(self.base + offset::GPIO_CLEARDATAOUT, 1 << pin);
        }
    }

    /// 핀 토글
    pub fn toggle(&self, pin: u8) {
        assert!(pin < 32);
        unsafe {
            let current = super::read_reg(self.base + offset::GPIO_DATAOUT);
            if current & (1 << pin) != 0 {
                self.set_low(pin);
            } else {
                self.set_high(pin);
            }
        }
    }

    /// 핀 입력값 읽기
    pub fn read(&self, pin: u8) -> bool {
        assert!(pin < 32);
        unsafe { super::read_reg(self.base + offset::GPIO_DATAIN) & (1 << pin) != 0 }
    }

    /// 인터럽트 활성화 (Rising Edge)
    pub fn enable_rising_irq(&self, pin: u8) {
        assert!(pin < 32);
        unsafe {
            super::set_bits(self.base + offset::GPIO_RISINGDETECT, 1 << pin);
            super::write_reg(self.base + offset::GPIO_IRQSTATUS_SET_0, 1 << pin);
        }
    }

    /// 인터럽트 상태 클리어
    pub fn clear_irq(&self, pin: u8) {
        assert!(pin < 32);
        unsafe {
            super::write_reg(self.base + offset::GPIO_IRQSTATUS_0, 1 << pin);
        }
    }
}
