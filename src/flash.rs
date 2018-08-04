use common::Constrain;
use stm32l0x1::{flash, FLASH};
use time::Hertz;

pub struct Flash {
    pub acr: ACR,
}

impl Constrain<Flash> for FLASH {
    fn constrain(self) -> Flash {
        Flash { acr: ACR(()) }
    }
}

impl Flash {
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

pub struct ACR(());

impl ACR {
    pub(crate) fn acr(&mut self) -> &flash::ACR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).acr }
    }

    pub(crate) fn flash_latency_1(&mut self) {
        self.acr().modify(|_, w| w.latency().set_bit());
        while self.acr().read().latency().bit_is_clear() {}
    }

    pub(crate) fn flash_latency_0(&mut self) {
        self.acr().modify(|_, w| w.latency().clear_bit());
        while self.acr().read().latency().bit_is_set() {}
    }
}
