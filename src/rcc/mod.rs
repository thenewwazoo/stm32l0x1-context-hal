//! STM32L0x1 Reset and Clock Control peripheral
//!
//! This module contains a partial abstraction over the RCC peripheral, as well as types and traits
//! related to the various clocks on the chip.

use common::Constrain;
use flash;
use power::{self, PowerContext};
use rcc::clocking::ClkSrc;
use stm32l0x1::{rcc, RCC};
use time::Hertz;

pub mod clocking;

impl Constrain<Rcc> for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb: AHB(()),
            apb1: APB1(()),
            apb2: APB2(()),
            iop: IOP(()),
            ccipr: CCIPR(()),
            cr: CR(()),
            csr: CSR(()),
            icscr: ICSCR(()),
            cfgr: CFGR {
                hclk_fclk: Hertz(2_097_000),
                pclk1: Hertz(2_097_000),
                pclk2: Hertz(2_097_000),
                sysclk_src: clocking::SysClkSource::MSI,
                lsi: clocking::LowSpeedInternalRC::new(),
                msi: clocking::MediumSpeedInternalRC::new(true, clocking::MsiFreq::Hz_2_097_000),
                hsi16: clocking::HighSpeedInternal16RC::new(),
                //pll: PLL,
                lse: None,
                //hse: Option<HighSpeedExternalOSC>,
            },
        }
    }
}

/// Struct representing the RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus (AHB) registers.
    pub ahb: AHB,
    /// APB1 peripheral registers.
    pub apb1: APB1,
    /// APB2 peripheral registers.
    pub apb2: APB2,
    /// GPIO port configuration
    pub iop: IOP,
    /// Peripherals independent clock configuration register
    pub ccipr: CCIPR,
    /// Clock control register
    pub cr: CR,
    /// Control/status register
    pub csr: CSR,
    /// HW clock configuration.
    pub cfgr: CFGR,
    /// Internal clock sources calibration register
    pub icscr: ICSCR,
}

impl Rcc {

    /// Configure the clocks and provide an operating context
    ///
    /// This method configures each clock according to the values set in the `cfgr` member of
    /// `Rcc`, and provides an operating context for that particular clock configuration.
    pub fn clock_domain<F>(
        &mut self,
        flash: &mut flash::Flash,
        pwr_ctx: &mut power::PowerContext,
        mut op: F,
    )
    where
        F: FnMut(ClockContext, &mut PowerContext),
    {
        let lsiclk = self.cfgr.lsi.configure(&mut self.csr);
        let msiclk = self.cfgr.msi.configure(&mut self.icscr, &mut self.cr);
        let hsi16clk = self.cfgr.hsi16.configure(&mut self.icscr, &mut self.cr);
        let lseclk = if let Some(lse) = self.cfgr.lse.as_ref() {
            lse.configure(&mut self.apb1, &mut self.cr, &mut self.csr, pwr_ctx)
        } else {
            None
        };

        let (sysclk, sw_bits) = match self.cfgr.sysclk_src {
            clocking::SysClkSource::MSI => (self.cfgr.msi.freq().unwrap(), 0b00),
            clocking::SysClkSource::HSI16 => (self.cfgr.hsi16.freq().unwrap(), 0b01),
            //clocking::SysClkSource::HSE => (self.cfgr.hse.freq().unwrap(), 0b10),
            //clocking::SysClkSource::PLLCLK => (self.cfgr.pll.freq().unwrap(), 0b11),
        };

        let (hpre_bits, hpre_ratio) = match sysclk.0 / self.cfgr.hclk_fclk.0 {
            0 => unreachable!(),
            1 => (0b0000, 1),
            2 => (0b1000, 2),
            3...5 => (0b1001, 4),
            6...11 => (0b1010, 8),
            12...39 => (0b1011, 16),
            40...95 => (0b1100, 64),
            96...191 => (0b1101, 128),
            192...383 => (0b1110, 256),
            _ => (0b1111, 512),
        };

        let hclk = sysclk.0 / hpre_ratio;

        let (ppre1_bits, ppre1_ratio) = match self.cfgr.hclk_fclk.0 / self.cfgr.pclk1.0 {
            0 => unreachable!(),
            1 => (0b000, 1),
            2 => (0b100, 2),
            3...5 => (0b101, 4),
            6...11 => (0b110, 8),
            _ => (0b111, 16),
        };

        let pclk1 = Hertz(hclk / ppre1_ratio);

        let (ppre2_bits, ppre2_ratio) = match self.cfgr.hclk_fclk.0 / self.cfgr.pclk2.0 {
            0 => unreachable!(),
            1 => (0b011, 1),
            2 => (0b100, 2),
            3...5 => (0b101, 4),
            6...11 => (0b110, 8),
            _ => (0b111, 16),
        };

        let pclk2 = Hertz(hclk / ppre2_ratio);
        let hclk = Hertz(hclk);

        flash.latency_domain(sysclk, || {
            self.cfgr.inner().write(|w| unsafe {
                w.ppre1()
                    .bits(ppre1_bits)
                    .ppre2()
                    .bits(ppre2_bits)
                    .hpre()
                    .bits(hpre_bits)
                    .sw()
                    .bits(sw_bits)
            });

            op(ClockContext {
                sysclk,
                hclk,
                pclk1,
                pclk2,
                lsiclk,
                msiclk,
                hsi16clk,
                lseclk,
                iop: &mut self.iop
            }, pwr_ctx);
        });
    }
}

/// "Frozen" clock context
pub struct ClockContext<'rcc> {
    /// Core system clock
    sysclk: Hertz,
    /// HCLK/FCLK
    hclk: Hertz,
    /// PCLK1 to APB1 peripherals
    pclk1: Hertz,
    /// PCLK2 to APB2 peripherals
    pclk2: Hertz,
    /// Low-speed internal RC clock speed
    lsiclk: Option<Hertz>,
    /// Medium-speed internal RC clock speed
    msiclk: Option<Hertz>,
    /// High-speed internal 16 MHz clock speed (optionally /4)
    hsi16clk: Option<Hertz>,
    /// Low-speed external 32kHz clock speed
    lseclk: Option<Hertz>,
    /// GPIO port configuration
    pub iop: &'rcc mut IOP,
}

impl<'rcc> ClockContext<'rcc> {
    /// Returns the frequency of the system clock SYSCLK, also CK_PWR
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns the frequency of HCLK/FCLK
    pub fn hclk_fclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of APB1
    pub fn apb1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of APB2
    pub fn apb2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns the frequency of the low speed internal RC, if turned on
    pub fn lsi(&self) -> Option<Hertz> {
        self.lsiclk
    }

    /// Returns the frequency of the medium speed internal RC, if turned on
    pub fn msi(&self) -> Option<Hertz> {
        self.msiclk
    }

    /// Returns the frequency of the high speed 16 MHz internal RC, if turned on
    pub fn hsi16(&self) -> Option<Hertz> {
        self.hsi16clk
    }

    /// Returns the frequency of the low speed external oscillator, if enabled
    pub fn lse(&self) -> Option<Hertz> {
        self.lseclk
    }
}

/// AHB 1-3 register access
pub struct AHB(());
impl AHB {
    /// Access AHB reset register
    pub fn rstr(&mut self) -> &rcc::AHBRSTR {
        unsafe { &(*RCC::ptr()).ahbrstr }
    }

    /// Access AHB clock enable register
    pub fn enr(&mut self) -> &rcc::AHBENR {
        unsafe { &(*RCC::ptr()).ahbenr }
    }

    /// Access AHB enable clock in sleep mode register
    pub fn smenr(&mut self) -> &rcc::AHBSMENR {
        unsafe { &(*RCC::ptr()).ahbsmenr }
    }
}

/// APB1 register access
pub struct APB1(());
impl APB1 {
    /// Access APB1RSTR1 reset register
    pub fn rstr(&mut self) -> &rcc::APB1RSTR {
        unsafe { &(*RCC::ptr()).apb1rstr }
    }

    /// Access APB1ENR reset register
    pub fn enr(&mut self) -> &rcc::APB1ENR {
        unsafe { &(*RCC::ptr()).apb1enr }
    }
}

/// APB2 register access
pub struct APB2(());
impl APB2 {
    /// Access APB2RSTR reset register
    pub fn rstr(&mut self) -> &rcc::APB2RSTR {
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
    /// Access APB2ENR reset register
    pub fn enr(&mut self) -> &rcc::APB2ENR {
        unsafe { &(*RCC::ptr()).apb2enr }
    }
}

/// I/O port register access
pub struct IOP(());
impl IOP {
    /// Access IOPENR enable register
    pub fn enr(&mut self) -> &rcc::IOPENR {
        unsafe { &(*RCC::ptr()).iopenr }
    }

    /// Access IOPRSTR reset register
    pub fn rstr(&mut self) -> &rcc::IOPRSTR {
        unsafe { &(*RCC::ptr()).ioprstr }
    }

    /// Access IOPSMENR sleep mode register
    pub fn smenr(&mut self) -> &rcc::IOPSMEN {
        unsafe { &(*RCC::ptr()).iopsmen }
    }
}

/// Clock configuration register
pub struct CCIPR(());
impl CCIPR {
    /// Direct access to RCC_CCIPR
    #[inline]
    pub fn inner(&mut self) -> &rcc::CCIPR {
        unsafe { &(*RCC::ptr()).ccipr }
    }
}

/// Clock control register
pub struct CR(());
impl CR {
    /// Direct access to RCC_CR
    #[inline]
    pub fn inner(&mut self) -> &rcc::CR {
        unsafe { &(*RCC::ptr()).cr }
    }
}

/// Control/status register
pub struct CSR(());
impl CSR {
    /// Direct access to RCC_CSR
    #[inline]
    pub fn inner(&mut self) -> &rcc::CSR {
        unsafe { &(*RCC::ptr()).csr }
    }
}

/// Clock configuration register
pub struct CFGR {
    /// Desired HCLK/FCLK frequency
    pub hclk_fclk: Hertz,
    /// Desired peripheral clock 1 frequency
    pub pclk1: Hertz,
    /// Desired peripheral clock 2 frequency
    pub pclk2: Hertz,
    /// SYSCLK source selection
    pub sysclk_src: clocking::SysClkSource,
    /// Low-speed internal RC configuration
    pub lsi: clocking::LowSpeedInternalRC,
    /// Medium-speed internal RC configuration
    pub msi: clocking::MediumSpeedInternalRC,
    /// High-speed 16 MHz internal RC configuration
    pub hsi16: clocking::HighSpeedInternal16RC,
    /// Low-speed external oscillator configuration
    pub lse: Option<clocking::LowSpeedExternalOSC>,
    //pub pll: clocking::PLL,
    //pub hse: Option<clocking::HighSpeedExternalOSC>,
}

/// Clock configuration register
impl CFGR {
    /// Direct access to RCC_CFGR
    #[inline]
    pub fn inner(&mut self) -> &rcc::CFGR {
        unsafe { &(*RCC::ptr()).cfgr }
    }
}

/// Internal clock sources calibration register
pub struct ICSCR(());
impl ICSCR {
    /// Direct access to RCC_ICSCR
    #[inline]
    pub fn inner(&mut self) -> &rcc::ICSCR {
        unsafe { &(*RCC::ptr()).icscr }
    }
}
