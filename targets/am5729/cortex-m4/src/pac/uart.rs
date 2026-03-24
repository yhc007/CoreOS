//! ─────────────────────────────────────────────────────────────
//! AM5729 UART 레지스터 정의
//! ─────────────────────────────────────────────────────────────
//! TRM SPRUHZ6 — Chapter 24: UART

/// UART 레지스터 오프셋 (16550 호환)
pub mod offset {
    pub const THR: u32 = 0x000;   // Transmit Holding (W)
    pub const RHR: u32 = 0x000;   // Receive Holding (R)
    pub const DLL: u32 = 0x000;   // Divisor Latch Low (LCR[7]=1)
    pub const IER: u32 = 0x004;   // Interrupt Enable
    pub const DLH: u32 = 0x004;   // Divisor Latch High (LCR[7]=1)
    pub const FCR: u32 = 0x008;   // FIFO Control (W)
    pub const IIR: u32 = 0x008;   // Interrupt Identification (R)
    pub const LCR: u32 = 0x00C;   // Line Control
    pub const MCR: u32 = 0x010;   // Modem Control
    pub const LSR: u32 = 0x014;   // Line Status
    pub const MDR1: u32 = 0x020;  // Mode Definition 1
    pub const SCR: u32 = 0x040;   // Supplementary Control
    pub const SSR: u32 = 0x044;   // Supplementary Status
    pub const SYSC: u32 = 0x054;  // System Configuration
    pub const SYSS: u32 = 0x058;  // System Status
}

/// LSR 비트
pub mod lsr {
    pub const DR: u32 = 1 << 0;       // Data Ready
    pub const THRE: u32 = 1 << 5;     // TX Holding Register Empty
    pub const TEMT: u32 = 1 << 6;     // Transmitter Empty
}

/// UART 드라이버
pub struct Uart {
    base: u32,
}

impl Uart {
    pub const unsafe fn new(base: u32) -> Self {
        Self { base }
    }

    /// UART 초기화 (921600 baud, 8N1)
    /// 48MHz 기준 = 48_000_000 / (16 * 921600) ≈ 3.25 → divisor = 3
    pub fn init_921600(&self) {
        unsafe {
            // 1. UART 비활성화
            super::write_reg(self.base + offset::MDR1, 0x07);

            // 2. LCR 설정 (DLAB=1로 divisor 접근)
            super::write_reg(self.base + offset::LCR, 0x83); // 8bit, no parity, 1 stop, DLAB=1

            // 3. Baud rate divisor 설정
            super::write_reg(self.base + offset::DLL, 3);  // 921600 baud @ 48MHz
            super::write_reg(self.base + offset::DLH, 0);

            // 4. LCR 설정 (DLAB=0)
            super::write_reg(self.base + offset::LCR, 0x03); // 8bit, no parity, 1 stop

            // 5. FIFO 활성화
            super::write_reg(self.base + offset::FCR, 0x07); // Enable + clear FIFO

            // 6. UART 16x 모드 활성화
            super::write_reg(self.base + offset::MDR1, 0x00);
        }
    }

    /// 1바이트 송신 (블로킹)
    pub fn write_byte(&self, byte: u8) {
        unsafe {
            // TX FIFO가 빌 때까지 대기
            while super::read_reg(self.base + offset::LSR) & lsr::THRE == 0 {}
            super::write_reg(self.base + offset::THR, byte as u32);
        }
    }

    /// 1바이트 수신 (블로킹)
    pub fn read_byte(&self) -> u8 {
        unsafe {
            // 데이터 준비될 때까지 대기
            while super::read_reg(self.base + offset::LSR) & lsr::DR == 0 {}
            super::read_reg(self.base + offset::RHR) as u8
        }
    }

    /// 데이터 수신 가능 여부
    pub fn data_available(&self) -> bool {
        unsafe { super::read_reg(self.base + offset::LSR) & lsr::DR != 0 }
    }

    /// 바이트 슬라이스 송신
    pub fn write_bytes(&self, data: &[u8]) {
        for &b in data {
            self.write_byte(b);
        }
    }

    /// rpmsg fallback: UART로 직접 통신 (rpmsg 불가 시)
    pub fn send_frame(&self, msg_type: u8, payload: &[u8]) {
        self.write_byte(0x02); // STX
        let len = payload.len() as u16;
        self.write_byte((len >> 8) as u8);
        self.write_byte((len & 0xFF) as u8);
        self.write_byte(msg_type);
        self.write_bytes(payload);
        self.write_byte(0x03); // ETX
    }
}
