//! Flash memory

use common::Constrain;
use stm32l0x1::{flash, FLASH};
use time::Hertz;

/// Onboard flash memory abstraction
pub struct Flash {
    /// FLASH_ACR register
    pub acr: ACR,
}

impl Constrain<Flash> for FLASH {
    fn constrain(self) -> Flash {
        Flash { acr: ACR(()) }
    }
}

impl Flash {
    /// Provide an operating context wherein the flash latency will not change.
    ///
    /// Because flash latency depends on core system clock, we want to make sure it does not
    /// change. This method provides an operating context for that.
    pub fn latency_domain<F>(&mut self, sysclk: Hertz, mut op: F)
    where
        F: FnMut(),
    {
        if sysclk.0 > 16_000_000 {
            self.acr.flash_latency_1();
        } else {
            self.acr.flash_latency_0();
        }

        op();
    }
}

/// Access control register
pub struct ACR(());

impl ACR {
    /// Direct access to FLASH_ACR
    pub(crate) fn acr(&mut self) -> &flash::ACR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).acr }
    }

    /// Set the flash latency to 1 clock cycle
    pub(crate) fn flash_latency_1(&mut self) {
        self.acr().modify(|_, w| w.latency().set_bit());
        while self.acr().read().latency().bit_is_clear() {}
    }

    /// Set the flash latency to 0 clock cycles
    pub(crate) fn flash_latency_0(&mut self) {
        self.acr().modify(|_, w| w.latency().clear_bit());
        while self.acr().read().latency().bit_is_set() {}
    }
}
