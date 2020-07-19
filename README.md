[![crates.io version](https://meritbadge.herokuapp.com/stm32f3xx-util)](https://crates.io/crates/stm32f3xx-util)
[![docs.rs](https://docs.rs/stm32f3xx-util/badge.svg)](https://docs.rs/stm32f3xx-util)

# Stm32f3xx-util

This crate contains utility code supporting features of the STM32f3 line that
haven't yet been merged into [stm32f3xx-hal](https://github.com/stm32-rs/stm32f3xx-hal).

Missing many features, but includes some useful ones.

Example using a GPIO interrupt:
```rust
#![no_main]
#![no_std]

use cortex_m::{self, interrupt::free};
use stm32f3xx_hal as hal;
use hal::{
    prelude::*,
    interrupt,
    stm32::{self, EXTI},
};

use stm32f3xx_util::{interrupt_, low_power, make_interrupt_handler};

fn main() {
    // Configure an interrupt triggered by PB3 being pulled to ground.
    interrupt_::setup_line(&mut dp.EXTI, 3, interrupt_::Edge::Falling);
    interrupt_::setup_gpio(&mut dp.SYSCFG, interrupt_::GpioReg::B, 3);
    make_interrupt_handler!(EXTI3);
}
```

Example using a RTC wakeup interrupt:
```rust
#![no_main]
#![no_std]

use cortex_m::{self, interrupt::free};
use stm32f3xx_hal as hal;
use hal::{
    prelude::*,
    interrupt,
    stm32::{self, EXTI},
};

use stm32f3xx_util::{interrupt_, make_interrupt_handler};

fn main() {
    // Configure an interrupt triggered by the RTC in 1 second.
    interrupt_::setup_rtc_wakeup(&mut dp.SYSCFG, exti: &mut dp.EXTI, rtc: &mut dp.RTC, 1_000);
    make_interrupt_handler!(RTC_WKUP);
}
```

Example initiating a low power state:
```rust
#![no_main]
#![no_std]

use cortex_m::{self, interrupt::free};
use stm32f3xx_hal as hal;
use hal::{
    prelude::*,
    interrupt,
    stm32::{self, EXTI},
};

use stm32f3xx_util::{interrupt_, low_power};

fn main() {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Set up microcontroller peripherals
    let mut dp = stm32::Peripherals::take().unwrap();

    // Execute one of these low-power state commands.
    low_power::sleep_now(&mut cp.SCB);
    // ...
    low_power::stop(&mut cp.SCB, &mut dp.PWR);
    // ...
    low_power::standby(&mut cp.SCB, &mut dp.PWR);

    // Enter the state by triggering a wait-for-interrupt command.
    cortex_m::asm::wfi();
}
```