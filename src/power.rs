//! Power configuration

use common::Constrain;
use cortex_m::{self, asm};
use rcc::{self, Rcc};
use stm32l0x1::{pwr, DBG, PWR};

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

    fn configure_power(&mut self, rcc: &mut Rcc, ulp: bool) {
        // Enable configuration of the PWR peripheral
        rcc.apb1.enr().modify(|_, w| w.pwren().set_bit());
        while !rcc.apb1.enr().read().pwren().bit_is_set() {}

        self.cr.inner().modify(|_, w| w.ulp().bit(ulp));

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
    }

    /// Operate within the currently configured power context
    ///
    /// This function provides an operating context for the MCU that guarantees a static power
    /// configuration. Because context moves self (and then returns/releases is), the power
    /// peripheral cannot be mutated from within the context-protected operation.
    pub fn power_domain<F>(&mut self, rcc: &mut Rcc, ulp: bool, mut op: F)
    where
        F: FnMut(&mut Rcc, PowerContext),
    {
        self.configure_power(rcc, ulp);

        op(
            rcc,
            PowerContext {
                vdd_range: self.vdd_range,
                vcore_range: self.get_vcore_range(),
                cr: &mut self.cr,
                //csr: &mut self.csr,
            },
        );
    }

    /// Freeze the power configuration. This is a "destructive" operation that will prevent you
    /// from (sanely) reconfiguring the power peripheral.
    pub fn freeze(mut self, rcc: &mut Rcc, ulp: bool) -> PowerConfig {
        self.configure_power(rcc, ulp);

        PowerConfig {
            vdd_range: self.vdd_range,
            vcore_range: self.get_vcore_range(),
        }
    }
}

/// Immutable context for operating in a configured power domain
pub struct PowerContext<'pwr> {
    /// The VDD range provided to the chip
    pub vdd_range: VddRange,
    /// The VCore range under which the chip currently operates
    pub vcore_range: VCoreRange,
    /// A control register handle used to provide RTC domain access
    cr: &'pwr mut CR,
    //csr: &'pwr mut CSR,
}

/// A frozen power peripheral configuration
pub struct PowerConfig {
    /// The VDD range provided to the chip
    pub vdd_range: VddRange,
    /// The VCore range under which the chip currently operates
    pub vcore_range: VCoreRange,
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

    /// Enter low-power SLEEP mode
    pub fn enter_low_power_sleep(
        &mut self,
        scb: &mut cortex_m::peripheral::SCB,
        sleep_method: SleepMethod,
        fast_wakeup: bool,
    ) {
        if cortex_m::peripheral::DCB::is_debugger_attached() {
            unsafe { &(*DBG::ptr()) }
                .cr
                .modify(|_, w| w.dbg_sleep().set_bit());
            // 27.9.1 When one of the DBG_STANDBY, DBG_STOP and DBG_SLEEP bit is set and the
            //    internal reference voltage is stopped in low-power mode (ULP bit set in
            //    PWR_CR register), then the Fast wakeup must be enabled (FWU bit set in
            //    PWR_CR).

            if self.cr.inner().read().ulp().bit_is_set() {
                self.cr.inner().modify(|_, w| w.fwu().set_bit());
            }
        } else {
            self.cr.inner().modify(|_, w| w.fwu().bit(fast_wakeup));
        }
        // NOTE `lpds` below is NOT LPDS, it's LPSDSR!!!
        self.cr
            .inner()
            .modify(|_, w| w.lpds().set_bit().lprun().set_bit().pdds().clear_bit());

        scb.clear_sleepdeep();

        match sleep_method {
            SleepMethod::WFI => asm::wfi(),
            SleepMethod::WFE => asm::wfe(),
            SleepMethod::SleepOnExit => unimplemented!(),
        };
    }

    /*
    /// Enter STOP mode
    ///
    /// TODO does not work
    pub fn enter_stop(
        &mut self,
        dcb: &mut cortex_m::peripheral::DCB,
        scb: &mut cortex_m::peripheral::SCB,
        sleep_method: SleepMethod,
        wake_clk: StopWakeClk,
        ulp: bool,
    ) {

        if dcb.is_debugger_attached() {
            unsafe { &(*DBG::ptr()) }
                .cr
                .modify(|_, w| w.dbg_stop().set_bit());
            if ulp {
                // 27.9.1 When one of the DBG_STANDBY, DBG_STOP and DBG_SLEEP bit is set and the
                //    internal reference voltage is stopped in low-power mode (ULP bit set in
                //    PWR_CR register), then the Fast wakeup must be enabled (FWU bit set in
                //    PWR_CR).
                unsafe { &(*PWR::ptr()) }.cr.modify(|_,w| w.fwu().set_bit());
            }
        }

        scb.set_sleepdeep();

        self.cr
            .inner()
            .modify(|_, w| w.cwuf().set_bit());
        while self.csr.inner().read().wuf().bit_is_set() {}

        self.cr.inner().modify(|_,w| w.lpds().clear_bit().pdds().clear_bit().ulp().bit(ulp));

        let rcc = unsafe { &(*RCC::ptr()) }; // Fuckery rating: 2/10. I do this to avoid having some kind of pwren_context or attaching this mechanism to a ClockContext.
        rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
        while rcc.apb1enr.read().pwren().bit_is_clear() {}

        match wake_clk {
            StopWakeClk::MSI => rcc.cfgr.modify(|_, w| w.stopwuck().clear_bit()),
            StopWakeClk::HSI16 => rcc.cfgr.modify(|_, w| w.stopwuck().set_bit()),
        };

        match sleep_method {
            SleepMethod::WFI => asm::wfi(),
            SleepMethod::WFE => asm::wfe(),
            SleepMethod::SleepOnExit => unimplemented!(),
        };

        scb.clear_sleepdeep();

        rcc.apb1enr.modify(|_, w| w.pwren().clear_bit());
        while rcc.apb1enr.read().pwren().bit_is_set() {}
    }
    */
}

/// Select the clock to use upon wake from stop mode
pub enum StopWakeClk {
    /// MSI
    MSI,
    /// HSI16
    HSI16,
}

/// How should the power peripheral sleep
pub enum SleepMethod {
    /// Use WFI to sleep
    WFI,
    /// Use WFE to sleep
    WFE,
    /// Set SLEEPONEXIT bit
    SleepOnExit,
}
