use cast::u8;

use stm32f3xx_hal::gpio::gpiob::{PB6, PB7};
use stm32f3xx_hal::gpio::AF4;
use stm32f3xx_hal::pac::I2C1;
use stm32f3xx_hal::rcc::{Clocks, APB1};
use stm32f3xx_hal::time::Hertz;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TxState {
    None,
    TxStart,
    TxReady,
    TxSent,
    TxComplete,
    TxStop,
    TxNack,
}

pub struct I2c1 {
    dev: I2C1,
    tx_state: TxState,
}

impl I2c1 {
    pub fn new(
        i2c: I2C1,
        _: (PB6<AF4>, PB7<AF4>),
        freq: Hertz,
        clocks: Clocks,
        apb1: &mut APB1,
    ) -> I2c1 {
        apb1.i2c_clock_enable();
        apb1.i2c_reset();

        let freq = freq.0;

        assert!(freq <= 1_000_000);

        // TODO review compliance with the timing requirements of I2C
        // t_I2CCLK = 1 / PCLK1
        // t_PRESC  = (PRESC + 1) * t_I2CCLK
        // t_SCLL   = (SCLL + 1) * t_PRESC
        // t_SCLH   = (SCLH + 1) * t_PRESC
        //
        // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
        // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
        let i2cclk = clocks.pclk1().0;
        let ratio = i2cclk / freq - 4;
        let (presc, scll, sclh, sdadel, scldel) = if freq >= 100_000 {
            // fast-mode or fast-mode plus
            // here we pick SCLL + 1 = 2 * (SCLH + 1)
            let presc = ratio / 387;

            let sclh = ((ratio / (presc + 1)) - 3) / 3;
            let scll = 2 * (sclh + 1) - 1;

            let (sdadel, scldel) = if freq > 400_000 {
                // fast-mode plus
                let sdadel = 0;
                let scldel = i2cclk / 4_000_000 / (presc + 1) - 1;

                (sdadel, scldel)
            } else {
                // fast-mode
                let sdadel = i2cclk / 8_000_000 / (presc + 1);
                let scldel = i2cclk / 2_000_000 / (presc + 1) - 1;

                (sdadel, scldel)
            };

            (presc, scll, sclh, sdadel, scldel)
        } else {
            // standard-mode
            // here we pick SCLL = SCLH
            let presc = ratio / 514;

            let sclh = ((ratio / (presc + 1)) - 2) / 2;
            let scll = sclh;

            let sdadel = i2cclk / 2_000_000 / (presc + 1);
            let scldel = i2cclk / 800_000 / (presc + 1) - 1;

            (presc, scll, sclh, sdadel, scldel)
        };

        let presc = u8(presc).unwrap();
        assert!(presc < 16);
        let scldel = u8(scldel).unwrap();
        assert!(scldel < 16);
        let sdadel = u8(sdadel).unwrap();
        assert!(sdadel < 16);
        let sclh = u8(sclh).unwrap();
        let scll = u8(scll).unwrap();

        // Configure for "fast mode" (400 KHz)
        // NOTE(write): writes all non-reserved bits.
        i2c.timingr.write(|w| {
            w.presc()
                .bits(presc)
                .scll()
                .bits(scll)
                .sclh()
                .bits(sclh)
                .sdadel()
                .bits(sdadel)
                .scldel()
                .bits(scldel)
        });

        I2c1 {
            dev: i2c,
            tx_state: TxState::None,
        }
    }

    pub fn enable(&mut self) {
        // Enable the peripheral
        self.dev.cr1.modify(|_, w| w.pe().set_bit());
    }

    pub fn enable_interrupts(&mut self) {
        self.dev.cr1.modify(|_, w| {
            w.errie()
                .enabled()
                .tcie()
                .enabled()
                .stopie()
                .enabled()
                .nackie()
                .enabled()
                .rxie()
                .enabled()
                .txie()
                .enabled()
        });
    }

    pub fn write_start(&mut self, addr: u8, n_bytes: u8) {
        self.dev.cr2.modify(|_, w| {
            w.sadd()
                .bits(u16::from(addr << 1))
                .rd_wrn()
                .write()
                .nbytes()
                .bits(n_bytes)
                .start()
                .start()
                .autoend()
                .automatic()
        });
        self.tx_state = TxState::TxStart;
    }

    pub fn isr_state(&mut self) -> TxState {
        let isr = self.dev.isr.read();
        if isr.nackf().bit() {
            self.tx_state = TxState::TxReady;
        } else if isr.stopf().bit() {
            self.tx_state = TxState::TxStop;
        } else if isr.tc().bit() {
            self.tx_state = TxState::TxComplete;
        } else if isr.txis().bit() && isr.txe().bit() {
            self.tx_state = TxState::TxReady;
        }
        self.tx_state
    }

    pub fn write_tx_buffer(&mut self, byte: u8) {
        self.dev.txdr.write(|w| w.txdata().bits(byte));
        self.tx_state = TxState::TxSent;
    }

    pub fn get_tx_state(&self) -> TxState {
        self.tx_state
    }

    pub fn reset_tx_state(&mut self) {
        self.tx_state = TxState::None;
    }

    pub fn clear_stop_int(&mut self) {
        self.dev.icr.write(|w| w.stopcf().bit(true));
    }

    pub fn get_isr(&self) -> u32 {
        self.dev.isr.read().bits()
    }

    pub fn get_txdr(&self) -> u32 {
        self.dev.txdr.read().bits()
    }
}
