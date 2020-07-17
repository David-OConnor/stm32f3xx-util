//! This module contains code used to place the STM32F3 in low power modes.
//! Reference section 3.7: `Power management` of the user manual.

/// Enter `Sleep now` mode: the lightest of the 3 low-power states avail on the
/// STM32f3.
/// TO exit: Interrupt. Refer to Table 82.
use stm32f3xx_hal::stm32;

pub fn sleep_now(scb: &mut cortex_m::peripheral::SCB) {
    // WFI (Wait for Interrupt) (eg `cortext_m::asm::wfi()) or WFE (Wait for Event) while:

    // SLEEPDEEP = 0 and SLEEPONEXIT = 0
    scb.clear_sleepdeep();
    // Sleep-now: if the SLEEPONEXIT bit is cleared, the MCU enters Sleep mode as soon
    // as WFI or WFE instruction is executed.
    scb.clear_sleeponexit();
}

/// Enter `Stop` mode: the middle of the 3 low-power states avail on the
/// STM32f3.
/// To exit:  Any EXTI Line configured in Interrupt mode (the corresponding EXTI
/// Interrupt vector must be enabled in the NVIC). Refer to Table 82.
pub fn stop(scb: &mut cortex_m::peripheral::SCB, pwr: &mut stm32::PWR) {
    //WFI (Wait for Interrupt) or WFE (Wait for Event) while:

    // Set SLEEPDEEP bit in ARM® Cortex®-M4 System Control register
    scb.set_sleepdeep();

    // Clear PDDS bit in Power Control register (PWR_CR)
    pwr.cr.modify(|_, w| w.pdds().clear_bit());

    // Select the voltage regulator mode by configuring LPDS bit in PWR_CR
    pwr.cr.modify(|_, w| w.lpds().set_bit()); // todo: Set or clear?
}

/// Enter `Standby` mode: the lowest-power of the 3 low-power states avail on the
/// STM32f3.
/// To exit: WKUP pin rising edge, RTC alarm event’s rising edge, external Reset in
/// NRST pin, IWDG Reset.
pub fn standby(scb: &mut cortex_m::peripheral::SCB, pwr: &mut stm32::PWR) {
    // WFI (Wait for Interrupt) or WFE (Wait for Event) while:

    // Set SLEEPDEEP bit in ARM® Cortex®-M4 System Control register
    scb.set_sleepdeep();

    // Set PDDS bit in Power Control register (PWR_CR)
    pwr.cr.modify(|_, w| w.pdds().set_bit());

    // Clear WUF bit in Power Control/Status register (PWR_CSR)
    //    pwr.cr.modify(|_, w| w.cwuf().clear_bit() );

    // https://vasiliev.me/blog/sending-stm32f1-to-deep-sleep-with-rust/
    let standby_flag = pwr.csr.read().sbf().bit();

    if standby_flag {
        // Clear standby flag
        pwr.cr.modify(|_, w| w.csbf().clear_bit());
        // Clear Wakeup flag
        pwr.cr.modify(|_, w| w.cwuf().set_bit());
    }
}
