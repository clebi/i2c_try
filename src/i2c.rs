use cast::u8;

use core::fmt;
use stm32f3xx_hal::gpio::gpiob::{PB6, PB7};
use stm32f3xx_hal::gpio::AF4;
use stm32f3xx_hal::pac::I2C1;
use stm32f3xx_hal::rcc::{Clocks, APB1};
use stm32f3xx_hal::time::Hertz;

/// I2c errors.
#[derive(Debug, Copy, Clone)]
pub enum I2cError {
    DeviceBusy, // Device is busy, can't start something else
    Nack,       // received a nack
    BusError,
    ArbitrationLoss,
    Overrun,
    StateError, // unable to compute the stop state because previous state was not expected
    TransferCompleteNoRead, // Tranfer complete status but nothing do do next
}

/// State of i2c communication.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Idle,
    TxStart,
    TxReady,
    TxSent,
    TxComplete,
    TxStop,
    RxStart,
    RxReady,
    RxStop,
    TxNack,
}

impl core::fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

/// I2c1 provides interface to communicate with i2c devices.
pub struct I2c1 {
    dev: I2C1,
    state: State,
    /// Last error that happened on i2c communications.
    pub last_error: Option<I2cError>,
    current_write_addr: Option<u8>,
    tx_ind: usize,
    tx_buf: Option<&'static [u8]>,
    rx_ind: usize,
    rx_buf: [u8; 256], // for now use a static buffer here.
    recv: Option<(u8, usize)>,
}

impl I2c1 {
    /// Creates the new i2c device.
    ///
    /// # Arguments
    /// * `i2c` - i2c device.
    /// * `freq` - frequency for the device.
    /// * `clocks` - systel clocks.
    /// * `apb1` - APB1 register.
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
            state: State::Idle,
            last_error: None,
            current_write_addr: None,
            tx_ind: 0,
            tx_buf: None,
            rx_ind: 0,
            rx_buf: [0; 256],
            recv: None,
        }
    }

    /// Enable the i2c device.
    pub fn enable(&mut self) {
        // Enable the peripheral
        self.dev.cr1.modify(|_, w| w.pe().set_bit());
    }

    /// Disable the i2c device.
    pub fn disable(&mut self) {
        self.dev.cr1.modify(|_, w| w.pe().disabled());
        self.last_error = None;
    }

    /// Enables all interrupts for i2c device.
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

    /// Disables all interrupts for i2c device.
    pub fn disable_interrupts(&mut self) {
        self.dev.cr1.modify(|_, w| {
            w.errie()
                .disabled()
                .tcie()
                .disabled()
                .stopie()
                .disabled()
                .nackie()
                .disabled()
                .rxie()
                .disabled()
                .txie()
                .disabled()
        });
    }

    /// Write bytes through i2c. Supports only write < 256 bytes.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    /// * `bytes` - Bytes to send.
    ///
    /// # Errors
    /// * `I2cError::DeviceBusy` if the device is already busy.
    pub fn write(&mut self, addr: u8, bytes: &'static [u8]) -> Result<(), I2cError> {
        self._write(addr, true, bytes)
    }

    /// Write bytes through i2c. Supports only write < 256 bytes.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    /// * `auto_stop` - i2c autostop enabled.
    /// * `bytes` - Bytes to send.
    ///
    /// # Errors
    /// * `I2cError::DeviceBusy` if the device is already busy.
    fn _write(&mut self, addr: u8, auto_stop: bool, bytes: &'static [u8]) -> Result<(), I2cError> {
        if self.is_busy() {
            self.last_error = Some(I2cError::DeviceBusy);
            return Err(I2cError::DeviceBusy);
        }
        self.tx_ind = 0;
        self.tx_buf = Some(bytes);
        self.write_start(addr, bytes.len() as u8, auto_stop);
        Ok(())
    }

    /// Start a write sequence on i2c channel.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    /// * `n_bytes` - number of bytes which will be sent.
    /// * `auto_stop` - i2c autostop enabled.
    fn write_start(&mut self, addr: u8, n_bytes: u8, auto_stop: bool) {
        self.current_write_addr = Some(addr);
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
                .bit(auto_stop)
        });
        self.state = State::TxStart;
    }

    /// Reads from i2c interface. Supports only read < 256 bytes.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    /// * `len` - number of bytes to read.
    pub fn read(&mut self, addr: u8, len: usize) {
        self.dev.cr2.modify(|_, w| {
            w.sadd()
                .bits(u16::from(addr << 1))
                .rd_wrn()
                .read()
                .nbytes()
                .bits(len as u8)
                .start()
                .start()
                .autoend()
                .automatic()
        });
        self.state = State::RxStart;
    }

    /// Write bytes through i2c. Supports only write and read < 256 bytes.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    /// * `bytes` - Bytes to send.
    /// * `recv_len` - Number of bytes to receive.
    ///
    /// # Errors
    /// * `I2cError::DeviceBusy` if the device is already busy.
    pub fn write_read(
        &mut self,
        addr: u8,
        bytes: &'static [u8],
        recv_len: usize,
    ) -> Result<(), I2cError> {
        self.recv = Some((addr, recv_len));
        self._write(addr, false, bytes)
    }

    pub fn stop(&mut self, addr: u8) {
        self.dev
            .cr2
            .modify(|_, w| w.sadd().bits(u16::from(addr << 1)).stop().stop());
    }

    /// This function must be called when there is an interruption on i2c device.
    /// It will compute the current state based on ISR register and execute work based on the state.
    pub fn interrupt(&mut self) {
        let isr_state = self.isr_state();
        match isr_state {
            Ok(State::TxReady) => {
                self.tx_buf
                    .map(|buf| self.write_tx_buffer(buf[self.tx_ind]));
                self.tx_ind += 1;
            }
            Ok(State::RxReady) => {
                self.rx_buf[self.rx_ind] = self.dev.rxdr.read().rxdata().bits();
                self.rx_ind += 1;
            }
            Ok(State::TxStop) => {
                self.tx_ind = 0;
                self.current_write_addr = None;
            }
            Ok(State::TxComplete) => {
                self.tx_ind = 0;
                self.last_error = self.recv.map_or_else(
                    || Some(I2cError::TransferCompleteNoRead),
                    |recv| {
                        self.read(recv.0, recv.1);
                        None
                    },
                );
                // if there was an error send a stop
                if self.last_error.is_some() {
                    self.current_write_addr.map(|addr| self.stop(addr));
                }
                self.current_write_addr = None;
            }
            Ok(State::RxStop) => {
                self.rx_ind = 0;
            }
            Ok(State::TxNack) => {
                self.tx_ind = 0;
                self.last_error = Some(I2cError::Nack);
                self.current_write_addr = None;
            }
            Err(err) => {
                self.last_error = Some(err);
                self.current_write_addr = None;
            }
            _ => {}
        }
    }

    /// Computes the states based on IRS register.
    fn isr_state(&mut self) -> Result<State, I2cError> {
        let isr = self.dev.isr.read();
        if isr.berr().bit() {
            self.dev.icr.write(|w| w.berrcf().bit(true));
            return Err(I2cError::BusError);
        } else if isr.arlo().bit() {
            self.dev.icr.write(|w| w.arlocf().bit(true));
            return Err(I2cError::ArbitrationLoss);
        } else if isr.ovr().bit() {
            self.dev.icr.write(|w| w.ovrcf().bit(true));
            return Err(I2cError::Overrun);
        }
        if isr.nackf().bit() {
            self.dev.icr.write(|w| w.nackcf().bit(true));
            self.state = State::TxNack;
        } else if isr.tc().bit() {
            self.state = State::TxComplete;
        } else if isr.txis().bit() && isr.txe().bit() {
            self.state = State::TxReady;
        } else if isr.rxne().bit() {
            self.state = State::RxReady;
        } else if isr.stopf().bit() {
            // clear stop bit once read
            self.dev.icr.write(|w| w.stopcf().bit(true));
            self.state = match self.state {
                State::TxSent | State::TxComplete => State::TxStop,
                State::RxReady => State::RxStop,
                _ => return Err(I2cError::StateError),
            }
        }
        Ok(self.state)
    }

    /// Write a bute into the tx buffer.
    fn write_tx_buffer(&mut self, byte: u8) {
        self.dev.txdr.write(|w| w.txdata().bits(byte));
        self.state = State::TxSent;
    }

    /// Is the device Budy ?
    /// # Returns
    /// true if budy, false if not.
    pub fn is_busy(&mut self) -> bool {
        self.dev.isr.read().busy().bit()
    }

    /// Get the state of the device.
    pub fn get_tx_state(&self) -> State {
        self.state
    }

    /// Get the content of the receive buffer.
    ///
    /// # Arguments
    /// * `len` - len of the slice to get.
    ///
    /// # Returns
    /// A slice containing of the rx buffer.
    pub fn get_rx_buf(&self, len: usize) -> &[u8] {
        self.rx_buf.split_at(len).0
    }

    /// Set the device state to idle.
    pub fn reset_state(&mut self) {
        self.state = State::Idle;
        self.last_error = None;
        self.tx_buf = None;
        self.recv = None;
    }

    /// Debug function which returns the content of the ISR register.
    pub fn get_isr(&self) -> u32 {
        self.dev.isr.read().bits()
    }
}
