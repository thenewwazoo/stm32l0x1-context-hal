
stm32l0x1-context-hal
=====================

This HAL for the STM32L0x1 family of chips is an effort to explore new ways of building HALs in Rust. It follows the same general outline as the "normal" HALs, with one exception: invalidation of peripherals is an explicit design goal.

The normal way
--------------

What does that mean in practice? In the other HALs patterned on Jorge Aparicio's [stm32f30x-hal](https://github.com/japaric/stm32f30x-hal) reference implementation of the [embedded-hal](https://docs.rs/embedded-hal/0.2.1/embedded_hal/), peripherals are consumed and "frozen". For example,

```rust
// Consume the chip peripherals in exchange for the HAL abstractions
let mut flash = d.FLASH.constrain();
let mut rcc = d.RCC.constrain();

// Configure the clocks as desired
rcc.cfgr.msi.set_freq(MsiFreq::Hz_131_072);
rcc.cfgr.sysclk_src = stm32l0x1_hal::rcc::clocking::SysClkSource::MSI;

let clocks = cfgr.freeze(&mut flash.acr,
    &mut rcc.apb1,
    &mut rcc.apb2,
    &mut rcc.icscr,
    &mut rcc.cr,
    &mut rcc.CSR,
    vcore_range: pwr.get_vcore_range(),
);

// After this point, the clock configuration cannot be changed
```

Once the configuration is set, the peripheral is frozen or otherwise consumed. After that, dependent peripherals can be constructed atop them. For example, USARTs:

```rust
// Select the clock we wish to use, with an immutable input clock speed that can never be changed:
let usart_clk_src = clocking::USARTClkSource::PCLK(
                        clocking::PeripheralClock::PCLK1(
                            clocks.pclk1()
                        )
                    );
// Now configure the USART based on that clock speed
let mut vcp_serial = Serial::usart2(
    d.USART2,
    (vcp_tx, vcp_rx),
    Bps(115_200_u32),
    usart_clk_src,
    &mut rcc.apb1,
    &mut rcc.ccipr,
);
```

If the clock speed ever were to change, the USART's configuration would be invalid. In this way Rust's move semantics give us safety. They also constrain us.

Another way
-----------

What if we could have guarantees that the UART would be invalidated and destroyed if the clocks were to change? Or, put another way, what if we could provide a bounded configuration domain in which the USART could be created? That is the purpose of this crate: to explore HAL design that lets us do things like reconfigure clock speeds on the fly.

Another, simpler example, is power domains. Clock speeds are limited by the core voltage of the CPU. In this crate, we provide the `power::Power::power_domain` method that gives us a `PowerContext` that represents the power configuration of the chip:

```rust
let mut pwr = d.PWR.constrain();

// Configure the Power peripheral to set the VDD. The VDD range has implications for the
// maximum clock speed one can run (higher voltage = higher MHz). As such, you want to make
// sure this cannot change unless you are prepared to invalidate your clock settings.
pwr.set_vcore_range(stm32l0x1_hal::power::VCoreRange::Range3)
    .unwrap();

pwr.power_domain(&mut rcc, true, |rcc, mut pwr_ctx| {
    // Within the closure, we are guaranteed that Vcore will not change, as to do so requires
    // mutating `pwr`! The PowerContext `pwr_ctx` contains information we need to make
    // decisions based on the VDD and VCore levels available to us. Furthermore, a PowerContext
    // can only be obtained within this closure, and is required by Rcc::clock_domain.

});
// Once we leave this power domain, we are free to reconfigure the power, for example, to raise it and permit higher clock speeds.
```

The resulting code ends up looking Javascript-y:

```rust
pwr.power_domain(&mut rcc, true, |rcc, mut pwr_ctx| {
    rcc.cfgr.msi.enable();
    rcc.cfgr.msi.set_freq(clocking::MsiFreq::Hz_131_072);
    rcc.cfgr.sysclk_src = stm32l0x1_hal::rcc::clocking::SysClkSource::MSI;

    rcc.clock_domain(&mut flash, &mut pwr_ctx, |clk_ctx, pwr_ctx| {
        // Within this closure, similarly to `power_domain`, we are assured that the clock
        // configuration cannot change. You cannot turn a clock on or off because we've
        // already mutably borrowed `rcc`. The `clk_ctx` provides clock speed information
        // that is useful to clocked peripheral busses.

        serial_port.serial_domain(
            (&mut tx_pin, &mut rx_pin),
            Bps(9600),
            clocking::USARTClkSource::SYSCLK,
            &clk_ctx,
            |mut tx, mut rx| {
                // It's closures all the way down. Here, we are assured that our output
                // pins won't change, and we don't have to move/consume the Serial port in
                // order to access tx and rx (which makes reclaiming the pins ... difficult).

                loop {
                    block!(tx.write(
                        block!(rx.read()).unwrap()
                        )).unwrap();
                }
            },
        );
    });
});

```

We can (and I do!) build loops, so I can raise and lower the clocks based on inputs.
