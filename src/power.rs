//! Power configuration and management

use core::marker::PhantomData;

use common::Constrain;
//use cortex_m::{self, asm};
use rcc;
use stm32l0x1::{pwr, PWR};
use time::Hertz;

// Why do I impl Constrain twice? Good question. The VDD range is physical property, and I can't
// pick the correct value for you. The VCore range and RTC enable state are set at startup time by
// the chip, so I can pick defaults based on that. It's admittedly a bit unergonomic, but you will
// need to specify the type of `T` for constrain:
//
// let pwr: Power<VddHigh, VCoreRange2, RtcDis> = d.PWR.constrain();
//
// The annotation is necessary because the spec for `fn constrain` does not permit us to say:
// ```rust
// impl Constrain<Power<VDD, VCoreRange2, RtcDis>> for PWR {
//     fn constrain<VDD>(self) -> Power<VDD, VCoreRange2, RtcDis> { // <-- fn type mis-match
// ```

impl Constrain<Power<VddLow, VCoreRange2, RtcDis>> for PWR {
    fn constrain(self) -> Power<VddLow, VCoreRange2, RtcDis> {
        Power {
            cr: CR(()),
            csr: CSR(()),
            _vdd: PhantomData,
            _vcore: PhantomData,
            _rtc: PhantomData,
        }
    }
}

impl Constrain<Power<VddHigh, VCoreRange2, RtcDis>> for PWR {
    fn constrain(self) -> Power<VddHigh, VCoreRange2, RtcDis> {
        Power {
            cr: CR(()),
            csr: CSR(()),
            _vdd: PhantomData,
            _vcore: PhantomData,
            _rtc: PhantomData,
        }
    }
}

pub struct Power<VDD, VCORE, RTC> {
    /// Power control register
    cr: CR,
    /// Power control status register
    csr: CSR,
    #[doc(hidden)]
    /// MCU VDD voltage range. This is external to the chip.
    _vdd: PhantomData<VDD>,
    #[doc(hidden)]
    /// MCU VCore voltage range. This is configurable.
    _vcore: PhantomData<VCORE>,
    #[doc(hidden)]
    /// RTC enable/disable state
    _rtc: PhantomData<RTC>,
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

/// 1.71 V - 3.6 V
pub struct VddHigh(());
/// 1.65 V - 3.6 V
pub struct VddLow(());

#[derive(PartialOrd, PartialEq)]
pub enum VCoreRange {
    Range3,
    Range2,
    Range1,
}

impl VCoreRange {
    pub fn bits(&self) -> u8 {
        match self {
            VCoreRange::Range1 => 0b01,
            VCoreRange::Range2 => 0b10,
            VCoreRange::Range3 => 0b11,
        }
    }
}

pub trait Vos {
    fn range() -> VCoreRange;
}

pub trait FreqLimit {
    fn max_freq() -> Hertz;
}

/// Range 1 is the "high performance" range. VCore = 1.8V
pub struct VCoreRange1(());
impl Vos for VCoreRange1 {
    fn range() -> VCoreRange {
        VCoreRange::Range1
    }
}

impl FreqLimit for VCoreRange1 {
    fn max_freq() -> Hertz {
        Hertz(32_000_00)
    }
}

/// Range 2 is the "medium performance" range. VCore = 1.5V
pub struct VCoreRange2(());
impl Vos for VCoreRange2 {
    fn range() -> VCoreRange {
        VCoreRange::Range2
    }
}

impl FreqLimit for VCoreRange2 {
    fn max_freq() -> Hertz {
        Hertz(16_000_00)
    }
}

/// Range 3 is the "low power" range. VCore = 1.2V
pub struct VCoreRange3(());
impl Vos for VCoreRange3 {
    fn range() -> VCoreRange {
        VCoreRange::Range3
    }
}

impl FreqLimit for VCoreRange3 {
    fn max_freq() -> Hertz {
        Hertz(4_200_00)
    }
}

// Here, we only permit calling into_vdd_range when the VCore range is range 2 or range 3. This is
// because, if the range is 1, the vdd range _must_ be High. The type system prevents us from
// constructing a Power<VddLow, VCoreRange1, RTC>.

impl<VDD, RTC> Power<VDD, VCoreRange2, RTC> {
    pub fn into_vdd_range<NEWVDD>(self) -> Power<VDD, VCoreRange2, RTC> {
        Power {
            cr: self.cr,
            csr: self.csr,
            _vdd: PhantomData,
            _vcore: PhantomData,
            _rtc: PhantomData,
        }
    }
}

impl<VDD, RTC> Power<VDD, VCoreRange3, RTC> {
    pub fn into_vdd_range<NEWVDD>(self) -> Power<VDD, VCoreRange3, RTC> {
        Power {
            cr: self.cr,
            csr: self.csr,
            _vdd: PhantomData,
            _vcore: PhantomData,
            _rtc: PhantomData,
        }
    }
}

impl<VCORE, RTC> Power<VddHigh, VCORE, RTC> {
    pub fn into_vcore_range<NEWRANGE>(self) -> Power<VddHigh, NEWRANGE, RTC>
    where
        NEWRANGE: Vos,
    {
        Power {
            cr: self.cr,
            csr: self.csr,
            _vdd: PhantomData,
            _vcore: PhantomData,
            _rtc: PhantomData,
        }
    }
}

impl<RTC> Power<VddLow, VCoreRange2, RTC> {
    pub fn into_vcore_range(self) -> Power<VddLow, VCoreRange3, RTC> {
        Power {
            cr: self.cr,
            csr: self.csr,
            _vdd: PhantomData,
            _vcore: PhantomData,
            _rtc: PhantomData,
        }
    }
}

impl<RTC> Power<VddLow, VCoreRange3, RTC> {
    pub fn into_vcore_range(self) -> Power<VddLow, VCoreRange2, RTC> {
        Power {
            cr: self.cr,
            csr: self.csr,
            _vdd: PhantomData,
            _vcore: PhantomData,
            _rtc: PhantomData,
        }
    }
}

impl<VDD, VCORE, RTC> Power<VDD, VCORE, RTC>
where
    VCORE: Vos,
{
    /// The PWREN bit must be set while changing the configuration of the power peripheral. This
    /// provides a context within which this is true.
    fn while_clk_en<F>(apb1: &mut rcc::APB1, mut op: F)
    where
        F: FnMut(),
    {
        // Enable configuration of the PWR peripheral
        apb1.enr().modify(|_, w| w.pwren().set_bit());
        while !apb1.enr().read().pwren().bit_is_set() {}

        op();

        apb1.enr().modify(|_, w| w.pwren().clear_bit());
        while apb1.enr().read().pwren().bit_is_set() {}
    }

    /// Provide a context for changing LSE and RTC settings.
    ///
    /// 7.3.20: Note: The LSEON, LSEBYP, RTCSEL, LSEDRV and RTCEN bits in the RCC control and
    ///   status register (RCC_CSR) are in the RTC domain. As these bits are write protected after
    ///   reset, the DBP bit in the Power control register (PWR_CR) has to be set to be able to
    ///   modify them. Refer to Section 6.1.2: RTC and RTC backup registers for further
    ///   information.
    pub fn dbp_context<F>(&mut self, mut op: F)
    where
        F: FnMut(),
    {
        self.cr.inner().modify(|_, w| w.dbp().set_bit());
        while self.cr.inner().read().dbp().bit_is_clear() {}

        op();

        self.cr.inner().modify(|_, w| w.dbp().clear_bit());
        while self.cr.inner().read().dbp().bit_is_set() {}
    }

    pub fn enact(&mut self, apb1: &mut rcc::APB1) {
        Power::<VDD, VCORE, RTC>::while_clk_en(apb1, || {
            self.cr
                .inner()
                .modify(|_, w| unsafe { w.vos().bits(VCORE::range().bits()) });

            // 4. Poll VOSF bit of in PWR_CSR register. Wait until it is reset to 0.
            while self.csr.inner().read().vosf().bit_is_set() {}
        });
    }

    pub fn read_vcore_range(&self) -> VCoreRange {
        match self.cr.inner().read().vos().bits() {
            0b01 => VCoreRange::Range1,
            0b10 => VCoreRange::Range2,
            0b11 => VCoreRange::Range3,
            _ => unreachable!(),
        }
    }
}

pub struct RtcEn(());
pub struct RtcDis(());

impl<VDD, VCORE> Power<VDD, VCORE, RtcEn>
where
    VCORE: Vos,
{
    pub fn disable_rtc(self, apb1: &mut rcc::APB1) -> Power<VDD, VCORE, RtcDis> {
        // (Disable the power interface clock)
        apb1.enr().modify(|_, w| w.pwren().clear_bit());
        while apb1.enr().read().pwren().bit_is_set() {}

        self.cr.inner().modify(|_, w| w.dbp().clear_bit());

        Power {
            cr: self.cr,
            csr: self.csr,
            _vdd: PhantomData,
            _vcore: PhantomData,
            _rtc: PhantomData,
        }
    }
}

impl<VDD, VCORE> Power<VDD, VCORE, RtcDis>
where
    VCORE: Vos,
{
    pub fn enable_rtc(
        mut self,
        cr: &mut rcc::CR,
        apb1: &mut rcc::APB1,
    ) -> Power<VDD, VCORE, RtcEn> {
        // From 6.4.1 bit 8 DBP:
        //  Note: If the HSE divided by 2, 4, 8 or 16 is used as the RTC clock, this bit must
        //  remain set to 1.
        if cr.inner().read().rtcpre().bits() == 0 {
            self.cr.inner().modify(|_, w| w.dbp().clear_bit());
        } else {
            self.cr.inner().modify(|_, w| w.dbp().set_bit());
        }

        // From 6.1.2 RTC registers access
        // 1. Enable the power interface clock by setting the PWREN bits in the RCC_APB1ENR register.
        apb1.enr().modify(|_, w| w.pwren().set_bit());
        while !apb1.enr().read().pwren().bit_is_set() {}

        // 2. Set the DBP bit in the PWR_CR register (see Section 6.4.1).
        // (note: or any other thing)
        self.dbp_context(|| {
            // 3. Select the RTC clock source through RTCSEL[1:0] bits in RCC_CSR register.
            // 4. Enable the RTC clock by programming the RTCEN bit in the RCC_CSR register.
        });

        Power {
            cr: self.cr,
            csr: self.csr,
            _vdd: PhantomData,
            _vcore: PhantomData,
            _rtc: PhantomData,
        }
    }
}
