//! Power configuration

use common::Constrain;
use rcc::{self, Rcc};
use stm32l0x1::{pwr, PWR};

impl Constrain<Power> for PWR {
    fn constrain(self) -> Power {
        Power {
            vdd_range: VddRange::High,
            vcore_range: VCoreRange::Range2,
            cr: CR(()),
            csr: CSR(()),
        }
    }
}

/// Constrained power peripheral
pub struct Power {
    /// MCU VDD voltage range. This is external to the chip.
    vdd_range: VddRange,
    /// MCU VCore voltage range. This is configurable.
    vcore_range: VCoreRange,
    /// Power control register
    cr: CR,
    /// Power control status register
    csr: CSR,
}

#[derive(PartialEq, Copy, Clone)]
/// Figure 11. Performance versus VDD and VCORE range
pub enum VddRange {
    /// 1.71 V - 3.6 V
    High,
    /// 1.65 V - 3.6 V
    Low,
}

#[derive(PartialEq, Copy, Clone)]
/// 6.1.4 Dynamic voltage scaling management
pub enum VCoreRange {
    /// Range 1 is the "high performance" range. Vcore = 1.8V
    Range1,
    /// Range 2 is the "medium performance" range. Vcore = 1.5V
    Range2,
    /// Range 3 is the "low power" range. Vcore = 1.2V
    Range3,
}

/// Failures related to power peripheral configuration
#[derive(Debug)]
pub enum PowerError {
    /// VDD is too low for the requested VCore range
    LowVDD,
}

/// Power control register
pub struct CR(());
impl CR {
    /// Direct access to PWR_CR
    #[inline]
    pub fn inner(&self) -> &pwr::CR {
        unsafe { &(*PWR::ptr()).cr }
    }
}

/// Power control/status register
pub struct CSR(());
impl CSR {
    /// Direct access to PWR_CR
    #[inline]
    pub fn inner(&self) -> &pwr::CSR {
        unsafe { &(*PWR::ptr()).csr }
    }
}

impl Power {
    /// Indicate the (externally-controlled) Vdd voltage range
    ///
    /// See 6.1 Power Supplies
    pub fn set_vdd_range(&mut self, r: VddRange) {
        self.vdd_range = r;
    }

    /// Set the Vcore voltage range
    pub fn set_vcore_range(&mut self, vcore_range: VCoreRange) -> Result<(), PowerError> {
        if vcore_range == VCoreRange::Range1 && self.vdd_range == VddRange::Low {
            return Err(PowerError::LowVDD);
        }

        self.vcore_range = vcore_range;

        Ok(())
    }

    /// Return the Vcore voltage range currently configured
    pub fn get_vcore_range(&self) -> VCoreRange {
        match unsafe { &(*PWR::ptr()) }.cr.read().vos().bits() {
            0b01 => VCoreRange::Range1,
            0b10 => VCoreRange::Range2,
            0b11 => VCoreRange::Range3,
            _ => panic!("invalid VOS"),
        }
    }

    /// Operate within the currently configured power context
    ///
    /// This function provides an operating context for the MCU that guarantees a static power
    /// configuration. Because context moves self (and then returns/releases is), the power
    /// peripheral cannot be mutated from within the context-protected operation.
    pub fn power_domain<F>(&mut self, rcc: &mut Rcc, mut op: F)
    where
        F: FnMut(&mut Rcc, PowerContext),
    {
        // Enable configuration of the PWR peripheral
        rcc.apb1.enr().modify(|_, w| w.pwren().set_bit());
        while !rcc.apb1.enr().read().pwren().bit_is_set() {}

        // Set VCore range. Procedure from sec 6.1.5 Dynamic voltage scaling configuration

        // 1. Check VDD to identify which ranges are allowed (see Figure 11: Performance versus VDD
        //    and VCORE range).
        // This is performed by set_vcore_range.

        // 2. Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0.
        while self.csr.inner().read().vosf().bit_is_set() {}

        let r = self.vcore_range;
        self.cr.inner().modify(|_, w| unsafe {
            // 3. Configure the voltage scaling range by setting the VOS[1:0] bits in the PWR_CR
            //    register.
            w.vos().bits(match r {
                VCoreRange::Range1 => 0b01,
                VCoreRange::Range2 => 0b10,
                VCoreRange::Range3 => 0b11,
            })
        });

        // 4. Poll VOSF bit of in PWR_CSR register. Wait until it is reset to 0.
        while self.csr.inner().read().vosf().bit_is_set() {}

        rcc.apb1.enr().modify(|_, w| w.pwren().clear_bit());
        while rcc.apb1.enr().read().pwren().bit_is_set() {}

        op(rcc, PowerContext {
            vdd_range: self.vdd_range,
            vcore_range: self.get_vcore_range(),
            cr: &self.cr,
        });
    }
}

/// Immutable context for operating in a configured power domain
pub struct PowerContext<'pwr> {
    /// The VDD range provided to the chip
    pub vdd_range: VddRange,
    /// The VCore range under which the chip currently operates
    pub vcore_range: VCoreRange,
    /// A control register handle used to provide RTC domain access
    //cr: &'pwr mut CR,
    cr: &'pwr CR,
}

impl<'pwr> PowerContext<'pwr> {
    /// Enable modification of register values in the RTC domain (see 7.3.20 RCC_CSR note).
    pub fn rtc_domain<F>(&mut self, apb1: &mut rcc::APB1, rcc_cr: &mut rcc::CR, mut op: F)
    where
        F: FnMut(),
    {
        // From 6.1.2 RTC registers access
        // 1. Enable the power interface clock by setting the PWREN bits in the RCC_APB1ENR register.
        apb1.enr().modify(|_, w| w.pwren().set_bit());
        while !apb1.enr().read().pwren().bit_is_set() {}

        // 2. Set the DBP bit in the PWR_CR register (see Section 6.4.1).
        self.cr.inner().modify(|_, w| w.dbp().set_bit());

        // 3. Select the RTC clock source through RTCSEL[1:0] bits in RCC_CSR register.
        // 4. Enable the RTC clock by programming the RTCEN bit in the RCC_CSR register.
        // (note: or any other thing)
        op();

        // From 6.4.1 bit 8 DBP:
        //  Note: If the HSE divided by 2, 4, 8 or 16 is used as the RTC clock, this bit must
        //  remain set to 1.
        if rcc_cr.inner().read().rtcpre().bits() == 0 {
            self.cr.inner().modify(|_, w| w.dbp().clear_bit());
        }

        // (Disable the power interface clock)
        apb1.enr().modify(|_, w| w.pwren().clear_bit());
        while apb1.enr().read().pwren().bit_is_set() {}
    }
}
