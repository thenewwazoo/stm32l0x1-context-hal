use power;
use rcc;
use time::Hertz;

pub trait ClkSrc {
    fn freq(&self) -> Option<Hertz>;
}

pub enum SysClkSource {
    MSI,
    HSI16,
    //HSE,
    //PLLCLK,
}

pub struct LowSpeedInternalRC {
    enable: bool,
}

impl LowSpeedInternalRC {
    /// Instantiate the LSI
    pub(crate) fn new() -> Self {
        LowSpeedInternalRC { enable: false }
    }

    /// Request that the LSI be enabled
    pub fn enable(&mut self) {
        self.enable = true
    }

    /// Request that the LSI be disabled
    pub fn disable(&mut self) {
        self.enable = false
    }

    /// Enable the LSI, and wait for it to become ready
    pub fn configure(&self, csr: &mut rcc::CSR) -> Option<Hertz> {
        if self.enable {
            csr.inner().modify(|_, w| w.lsion().set_bit());
            while csr.inner().read().lsion().bit_is_clear() {}
        } else {
            csr.inner().modify(|_, w| w.lsion().clear_bit());
            while csr.inner().read().lsion().bit_is_set() {}
        }
        self.freq()
    }
}

impl ClkSrc for LowSpeedInternalRC {
    fn freq(&self) -> Option<Hertz> {
        if self.enable {
            Some(Hertz(37_000))
        } else {
            None
        }
    }
}

/// Onboard medium-speed internal RC clock source
pub struct MediumSpeedInternalRC {
    /// Request that the the MSI RC be enabled/disabled
    enable: bool,
    /// The requested MSI RC frequency
    freq: MsiFreq,
}

impl MediumSpeedInternalRC {
    /// Create a new MSI RC instance
    pub (crate) fn new(enable: bool, freq: MsiFreq) -> Self {
        MediumSpeedInternalRC { enable, freq }
    }

    /// Request that the MSI be enabled
    pub fn enable(&mut self) {
        self.enable = true;
    }

    /// Request that the MSI be disabled
    pub fn disable(&mut self) {
        self.enable = false
    }

    /// Set the desired MSI frequency range
    pub fn set_freq(mut self, f: MsiFreq) -> Self {
        self.freq = f;
        self
    }

    /// Convert the freq range to MSIRANGE bits (7.3.2)
    pub fn bits(&self) -> u8 {
        self.freq as u8
    }

    /// Configures the MSI to the specified frequency
    pub (crate) fn configure(&self, icscr: &mut rcc::ICSCR, cr: &mut rcc::CR) -> Option<Hertz> {
        if self.enable {
            icscr
                .inner()
                .modify(|_, w| unsafe { w.msirange().bits(self.bits()) });
            cr.inner().modify(|_, w| w.msion().set_bit());
            while cr.inner().read().msirdy().bit_is_clear() {}
        } else {
            cr.inner().modify(|_, w| w.msion().clear_bit());
            while cr.inner().read().msirdy().bit_is_set() {}
        }
        self.freq()
    }
}

impl ClkSrc for MediumSpeedInternalRC {
    /// Retrieve the desired MSI RC frequency range
    fn freq(&self) -> Option<Hertz> {
        if self.enable {
            Some(match self.freq {
                MsiFreq::Hz_65_536 => Hertz(65_536),
                MsiFreq::Hz_131_072 => Hertz(131_072),
                MsiFreq::Hz_262_144 => Hertz(262_144),
                MsiFreq::Hz_524_288 => Hertz(524_288),
                MsiFreq::Hz_1_048_000 => Hertz(1_048_000),
                MsiFreq::Hz_2_097_000 => Hertz(2_097_000),
                MsiFreq::Hz_4_194_000 => Hertz(4_194_000),
            })
        } else {
            None
        }
    }
}

#[repr(u8)]
#[derive(Copy, Clone)]
#[allow(non_camel_case_types)]
/// Available MSI RC frequency ranges
pub enum MsiFreq {
    Hz_65_536 = 0b000,
    Hz_131_072 = 0b001,
    Hz_262_144 = 0b010,
    Hz_524_288 = 0b011,
    Hz_1_048_000 = 0b100,
    Hz_2_097_000 = 0b101,
    Hz_4_194_000 = 0b110,
}

/// Onboard high-speed internal RC clock source
pub struct HighSpeedInternal16RC {
    /// Request that the HSI16 be enabled/disabled
    enable: bool,
    /// Should the HSI16 clock be prescaled by 4
    div4: bool,
}

impl HighSpeedInternal16RC {
    /// Instantiate a new HSI16
    pub (crate) fn new() -> Self {
        HighSpeedInternal16RC {
            enable: false,
            div4: false,
        }
    }

    /// Request that the HSI16 RC be enabled
    pub fn enable(&mut self) {
        self.enable = true;
    }

    /// Request that the HSI16 RC be disabled
    pub fn disable(&mut self) {
        self.enable = false
    }

    /// Request that the HSI16 clock be divided by 4
    pub fn div4(&mut self) {
        self.div4 = true;
    }

    /// Request that the HSI16 clock not be divided
    pub fn no_div(&mut self) {
        self.div4 = false;
    }

    /// Applies the selection options to the configuration registers and turns the clock on
    pub(crate) fn configure(&self, icscr: &mut rcc::ICSCR, cr: &mut rcc::CR) -> Option<Hertz> {
        if self.enable {
            icscr
                .inner()
                .modify(|_, w| unsafe { w.hsi16trim().bits(0x10) }); // 16 is the default value
            cr.inner().modify(|_, w| w.hsi16on().set_bit());
            while cr.inner().read().hsi16rdyf().bit_is_clear() {}

            if self.div4 {
                cr.inner().modify(|_, w| w.hsi16diven().set_bit());
                while cr.inner().read().hsi16divf().bit_is_clear() {}
            } else {
                cr.inner().modify(|_, w| w.hsi16diven().clear_bit());
                while cr.inner().read().hsi16divf().bit_is_set() {}
            }
        } else {
            cr.inner().modify(|_, w| w.hsi16on().clear_bit());
            while cr.inner().read().hsi16rdyf().bit_is_set() {}
        }
        self.freq()
    }
}

impl ClkSrc for HighSpeedInternal16RC {
    /// Retrieve the desired HSI16 RC frequency
    fn freq(&self) -> Option<Hertz> {
        if self.enable {
            if self.div4 {
                Some(Hertz(4_000_000))
            } else {
                Some(Hertz(16_000_000))
            }
        } else {
            None
        }
    }
}

#[derive(Default)]
/// Optional external low-speed 32 kHz oscillator
pub struct LowSpeedExternalOSC {
    /// Indicate that the LSE should be turned on
    enable: bool,
}

impl LowSpeedExternalOSC {
    /// Create a new LSE
    pub fn new() -> Self {
        LowSpeedExternalOSC { enable: true }
    }

    /// Indicate that the LSE should be turned on
    pub fn enable(&mut self) {
        self.enable = true;
    }

    /// Indicate that the LSE should not be turned on
    pub fn disable(&mut self) {
        self.enable = false;
    }

    /// Enable the LSE, and wait for it to become ready
    pub (crate) fn configure(
        &self,
        apb1: &mut rcc::APB1,
        cr: &mut rcc::CR,
        csr: &mut rcc::CSR,
        pwr_ctx: &mut power::PowerContext,
    ) -> Option<Hertz> {
        if self.enable {
            pwr_ctx.rtc_domain(apb1, cr, || {
                csr.inner().modify(|_, w| w.lseon().set_bit());
                while csr.inner().read().lseon().bit_is_clear() {}
            });
        } else {
            pwr_ctx.rtc_domain(apb1, cr, || {
                csr.inner().modify(|_, w| w.lseon().clear_bit());
                while csr.inner().read().lseon().bit_is_set() {}
            });
        }
        self.freq()
    }
}

impl ClkSrc for LowSpeedExternalOSC {
    /// Retrieve the LSE frequency
    fn freq(&self) -> Option<Hertz> {
        if self.enable {
            Some(Hertz(32_768))
        } else {
            None
        }
    }
}