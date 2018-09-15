//! Inter-Integrated Circuit (I2C) bus

use stm32l0x1::{I2C1, I2C2, I2C3, RCC};

use hal::blocking::i2c::{Write, WriteRead};
use rcc::APB1;
use time::Hertz;

/// Available clock sources for I2C modules
pub enum I2cClkSrc {
    /// APB1
    PCLK1(Hertz),
    /// High-speed internal 16 MHz
    HSI16(Hertz),
    /// SYSCLK
    Sysclk(Hertz),
}

impl I2cClkSrc {
    /// Get the frequency of the incoming clock
    fn freq(&self) -> Hertz {
        match *self {
            I2cClkSrc::PCLK1(f) => f,
            I2cClkSrc::HSI16(f) => f,
            I2cClkSrc::Sysclk(f) => f,
        }
    }
}

/// I2C error
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
    #[doc(hidden)]
    _Extensible,
}

// FIXME these should be "closed" traits
/// SCL pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SclPin<I2C> {}

/// SDA pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SdaPin<I2C> {}

#[cfg(any(feature = "STM32L011x3", feature = "STM32L011x4"))]
/// I2C pin definitions for the STM32L011x3/4 family
pub mod stm32l011x3_4;

#[cfg(any(feature = "STM32L031x4", feature = "STM32L031x6"))]
/// I2C pin definitions for the STM32L031x4/6 family
pub mod stm32l031x4_6;

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if isr.$flag().bit_is_set() {
                break;
            } else {
                // try again
            }
        }
    };
}

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident, $i2cXen:ident, $i2cXrst:ident, $i2cXsel0:ident, $i2cXsel1:ident),)+) => {
        $(
            impl<SCL, SDA> I2c<$I2CX, (SCL, SDA)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F>(
                    i2c: $I2CX,
                    pins: (SCL, SDA),
                    freq: F,
                    clk_src: I2cClkSrc,
                    apb1: &mut APB1,
                ) -> Self where
                    F: Into<Hertz>,
                    SCL: SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    apb1.enr().modify(|_, w| w.$i2cXen().set_bit());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().set_bit());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().clear_bit());

                    let (i2cclk, sel0_bit, sel1_bit) = match clk_src {
                        I2cClkSrc::PCLK1(f) => (f.0, false, false),
                        I2cClkSrc::Sysclk(f) => (f.0, false, true),
                        I2cClkSrc::HSI16(f) => (f.0, true, true),
                    };

                    // This is weird and bad, but the fact that the I2C has to reach back into
                    // the RCC peripheral in order to configure its own clock is ... unfortunate.
                    // Perhaps I could TODO expose the appropriate configuration method through
                    // clk_ctx, but that'd require a mutable borrow and ... well.
                    //
                    // note for future: see hanno braun's implementation of AnalogBlock trait.
                    // Might work well here.
                    let ccipr = unsafe { &(*RCC::ptr()).ccipr };
                    ccipr.modify(|_,w| w.$i2cXsel0().bit(sel0_bit).$i2cXsel1().bit(sel1_bit));

                    i2c.cr1.write(|w| w.pe().clear_bit());

                    let freq = freq.into().0;

                    assert!(freq <= 100_000);

                    // TODO review compliance with the timing requirements of I2C
                    // t_I2CCLK = 1 / PCLK1
                    // t_PRESC  = (PRESC + 1) * t_I2CCLK
                    // t_SCLL   = (SCLL + 1) * t_PRESC
                    // t_SCLH   = (SCLH + 1) * t_PRESC
                    //
                    // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
                    // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
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

                    let presc = presc as u8;
                    assert!(presc < 16);
                    let scldel = scldel as u8;
                    assert!(scldel < 16);
                    let sdadel = sdadel as u8;
                    assert!(sdadel < 16);
                    let sclh = sclh as u8;
                    let scll = scll as u8;

                    // Configure for "fast mode" (400 KHz)
                    i2c.timingr.write(|w| unsafe {
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

                    // Enable the peripheral
                    i2c.cr1.write(|w| w.pe().set_bit());

                    I2c { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Write for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

                    // START and prepare to send `bytes`
                    self.i2c.cr2.write(|w| unsafe {
                        w
                            .sadd()
                            .bits(addr as u16)
                            .rd_wrn()
                            .clear_bit()
                            .nbytes()
                            .bits(bytes.len() as u8)
                            .start()
                            .set_bit()
                            .autoend()
                            .set_bit()
                    });

                    for byte in bytes {
                        // Wait until we are allowed to send data (START has been ACKed or last byte
                        // when through)
                        busy_wait!(self.i2c, txis);

                        // put byte on the wire
                        unsafe { self.i2c.txdr.write(|w| w.txdata().bits(*byte)) };
                    }

                    // Wait until the last transmission is finished ???
                    // busy_wait!(self.i2c, busy);

                    // automatic STOP

                    Ok(())
                }
            }

            impl<PINS> WriteRead for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write_read(
                    &mut self,
                    addr: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);
                    assert!(buffer.len() < 256 && buffer.len() > 0);

                    // TODO do we have to explicitly wait here if the bus is busy (e.g. another
                    // master is communicating)?

                    // START and prepare to send `bytes`
                    self.i2c.cr2.write(|w| unsafe {
                        w
                            .sadd()
                            .bits(addr as u16)
                            .rd_wrn()
                            .clear_bit()
                            .nbytes()
                            .bits(bytes.len() as u8)
                            .start()
                            .set_bit()
                            .autoend()
                            .clear_bit()
                    });

                    for byte in bytes {
                        // Wait until we are allowed to send data (START has been ACKed or last byte
                        // when through)
                        busy_wait!(self.i2c, txis);

                        // put byte on the wire
                        unsafe { self.i2c.txdr.write(|w| w.txdata().bits(*byte)) };
                    }

                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, tc);

                    // reSTART and prepare to receive bytes into `buffer`
                    self.i2c.cr2.write(|w| unsafe {
                        w
                            .sadd()
                            .bits(addr as u16)
                            .rd_wrn()
                            .set_bit()
                            .nbytes()
                            .bits(buffer.len() as u8)
                            .start()
                            .set_bit()
                            .autoend()
                            .set_bit()
                    });

                    for byte in buffer {
                        // Wait until we have received something
                        busy_wait!(self.i2c, rxne);

                        *byte = self.i2c.rxdr.read().rxdata().bits();
                    }

                    // automatic STOP

                    Ok(())
                }
            }
        )+
    }
}

hal! {
    I2C1: (i2c1, i2c1en, i2c1rst, i2c1sel0, i2c1sel1),
}