//! Configure interrupts for the stm32f3xx. Release as a standalone module
//! once complete.
//!
//! See Jayce Boyd's guide here:
//!     https://github.com/jkboyce/stm32f3xx-hal/blob/master/examples/gpio_interrupt.rs
//!     See ref manual page 249+
//!
//! Special thanks to Jayce Boyd and Adam Greig, for being super bros.

use cortex_m::peripheral::NVIC;
use stm32f3xx_hal::{
    interrupt,
    rcc::APB1,
    stm32::{EXTI, PWR, RTC, SYSCFG},
};

// use paste;

#[derive(Copy, Clone, Debug)]
pub enum GpioReg {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
}

impl GpioReg {
    /// See ref manual section 12.1.3: each reg has an associated value
    fn cr_val(&self) -> u8 {
        match self {
            Self::A => 0,
            Self::B => 1,
            Self::C => 2,
            Self::D => 3,
            Self::E => 4,
            Self::F => 5,
            Self::G => 6,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum PNum {
    One,
    Two,
    Three,
}
// todo: Just use a u8 etc for this?

// impl GpioReg {
//     fn cr(&self) -> Cr {
//         match self {
//             Self::PA1 => Cr1,
//             Self::PA2 => Cr1,
//         }
//     }
// }

// #[derive(Copy, Clone, Debug)]
// pub enum Task {
//     Enable,
//     Disable,
// }

#[derive(Copy, Clone, Debug)]
pub enum Edge {
    Rising,
    Falling,
}

/// Set EWUP bit in PWR_CSR register; set PWREN bit to enable power interface clock.
/// This is required to enable use of the WKUP pin, eg to wake up from standby.
/// https://vasiliev.me/blog/sending-stm32f1-to-deep-sleep-with-rust/#fn:3
/// We use our custom stm32f3xx hal branch to allow access to the (normally private) `enr`.
pub fn setup_wakeup(apb1: &mut APB1, pwr: &mut PWR) {
    // todo: We must somehow call this too:
    // enable SYSCFG clock to enable external interrupts; must come before RCC.constrain()
    // dp.RCC.apb2enr.write(|w| w.syscfgen().enabled());

    apb1.enr().modify(|_, w| w.pwren().set_bit());
    pwr.csr.modify(|_, w| w.ewup1().set_bit());
}

/// These functions are called when their appropriate interrupt line
/// is triggered.
/// Eg:
///     make_interrupt_handler!(EXTI3);
///     make_interrupt_handler!(RTC_WKUP);
///     make_interrupt_handler!(EXTI15_10);
#[macro_export]
macro_rules! make_interrupt_handler {
    ($line:ident) => {
        #[interrupt]
        fn $line() {
            free(|cs| {
                // Reset pending bit for interrupt line
                // todo: pr1 for all?
                unsafe { (*EXTI::ptr()).pr1.modify(|_, w| w.pr1().bit(true)) };
                // hprintln!("In exti $line").ok();
            });
        }
    };
}

//
// #[macro_use]
// macro_rules! set_interrupt {
//     ($line:expr, $fnname:ident) => {
//         pub fn $fnname(
//             dp: &mut stm32::Peripherals,
//             rcc: &mut stm32::RCC,
//             p_reg: GpioReg,
//             task: Task,
//             edge: Edge,
//         ) {
//             // Select this GPIO pin as source input for EXTI$line external interrupt
//
//             // if line$ <= 3 {
//                 dp.SYSCFG
//                     .exticr1
//                     .modify(|_, w| unsafe { w.exti$line().bits(gpio_reg.cr_val()) });
//             // } else if line$ <= 7 {
//             //     dp.SYSCFG
//             //         .exticr2
//             //         .modify(|_, w| unsafe { w.exti$line().bits(gpio_reg.cr_val()) });
//             // } else if line$ <= 11 {
//             //     dp.SYSCFG
//             //         .exticr3
//             //         .modify(|_, w| unsafe { w.exti$line().bits(gpio_reg.cr_val()) });
//             // }
//             // // todo etc.
//
//             match edge {
//                 Edge::Rising => {
//                     // configure EXTI$line to trigger on rising edge, disable trigger on falling edge
//                     dp.EXTI.rtsr1.modify(|_, w| unsafe { w.tr$line().bit(true) });
//                     dp.EXTI.ftsr1.modify(|_, w| unsafe { w.tr$line().bit(false) });
//                 }
//                 Edge::Falling => {
//                     // configure EXTI$line to trigger on falling edge, disable trigger on rising edge
//                     dp.EXTI.rtsr1.modify(|_, w| unsafe { w.tr$line().bit(false) });
//                     dp.EXTI.ftsr1.modify(|_, w| unsafe { w.tr$line().bit(true) });
//                 }
//             }
//
//             // unmask EXTI
//             dp.EXTI.imr1.modify(|_, w| w.mr$line().unmasked());
//
//             unsafe { NVIC::unmask(interrupt::EXTI$line) };
//         }
//     };
// }

/// Reference the STM32F303 reference manual, section 14.2.5 for a `Functional description`
/// of the steps we accomplish here.
/// 0 - 15.
pub fn setup_line(exti: &mut EXTI, line: u8, edge: Edge) {
    // Configure the Trigger Selection bits of the Interrupt line (EXTI_RTSR and EXTI_FTSR)
    let rise_trigger = match edge {
        Edge::Rising => {
            // configure EXTI line to trigger on rising edge, disable trigger on falling edge.
            true
        }
        Edge::Falling => {
            // configure EXTI$line to trigger on falling edge, disable trigger on rising edge
            false
        }
    };

    match line {
        // External, eg GPIO interrupts
        0 => {
            // Configure the corresponding mask bit in the EXTI_IMR register.
            exti.imr1.modify(|_, w| w.mr0().unmasked());
            exti.rtsr1.modify(|_, w| w.tr0().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr0().bit(!rise_trigger));

            // Configure the enable and mask bits that control the NVIC IRQ channel mapped to the
            // EXTI so that an interrupt coming from one of the EXTI line can be correctly
            // acknowledged.
            // See reference manual Table 83.
            unsafe { NVIC::unmask(interrupt::EXTI0) };
        }
        // todo: DRY, in lieu of a working macro.
        1 => {
            exti.imr1.modify(|_, w| w.mr1().unmasked());
            exti.rtsr1.modify(|_, w| w.tr1().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr1().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI1) };
        }
        2 => {
            exti.imr1.modify(|_, w| w.mr2().unmasked());
            exti.rtsr1.modify(|_, w| w.tr2().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr2().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI2_TSC) };
        }
        3 => {
            exti.imr1.modify(|_, w| w.mr3().unmasked());
            exti.rtsr1.modify(|_, w| w.tr3().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr3().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI3) };
        }
        4 => {
            exti.imr1.modify(|_, w| w.mr4().unmasked());
            exti.rtsr1.modify(|_, w| w.tr4().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr4().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI4) };
        }
        5 => {
            exti.imr1.modify(|_, w| w.mr5().unmasked());
            exti.rtsr1.modify(|_, w| w.tr5().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr5().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI9_5) };
        }
        6 => {
            exti.imr1.modify(|_, w| w.mr6().unmasked());
            exti.rtsr1.modify(|_, w| w.tr6().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr6().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI9_5) };
        }
        7 => {
            exti.imr1.modify(|_, w| w.mr7().unmasked());
            exti.rtsr1.modify(|_, w| w.tr7().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr7().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI9_5) };
        }
        8 => {
            exti.imr1.modify(|_, w| w.mr8().unmasked());
            exti.rtsr1.modify(|_, w| w.tr8().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr8().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI9_5) };
        }
        9 => {
            exti.imr1.modify(|_, w| w.mr9().unmasked());
            exti.rtsr1.modify(|_, w| w.tr9().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr9().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI9_5) };
        }
        10 => {
            exti.imr1.modify(|_, w| w.mr10().unmasked());
            exti.rtsr1.modify(|_, w| w.tr10().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr10().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI15_10) };
        }
        11 => {
            exti.imr1.modify(|_, w| w.mr11().unmasked());
            exti.rtsr1.modify(|_, w| w.tr11().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr11().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI15_10) };
        }
        12 => {
            exti.imr1.modify(|_, w| w.mr12().unmasked());
            exti.rtsr1.modify(|_, w| w.tr12().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr12().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI15_10) };
        }
        13 => {
            exti.imr1.modify(|_, w| w.mr13().unmasked());
            exti.rtsr1.modify(|_, w| w.tr13().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr13().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI15_10) };
        }
        14 => {
            exti.imr1.modify(|_, w| w.mr14().unmasked());
            exti.rtsr1.modify(|_, w| w.tr14().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr14().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI15_10) };
        }
        // Internal interrupts
        15 => {
            exti.imr1.modify(|_, w| w.mr15().unmasked());
            exti.rtsr1.modify(|_, w| w.tr15().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr15().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::EXTI15_10) };
        }
        16 => {
            exti.imr1.modify(|_, w| w.mr16().unmasked());
            exti.rtsr1.modify(|_, w| w.tr16().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr16().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::PVD) };
        }
        17 => {
            exti.imr1.modify(|_, w| w.mr17().unmasked());
            exti.rtsr1.modify(|_, w| w.tr17().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr17().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::RTCALARM) };
        }
        18 => {
            exti.imr1.modify(|_, w| w.mr18().unmasked());
            exti.rtsr1.modify(|_, w| w.tr18().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr18().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::USB_WKUP) };
        }
        19 => {
            exti.imr1.modify(|_, w| w.mr19().unmasked());
            exti.rtsr1.modify(|_, w| w.tr19().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr19().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::TAMP_STAMP) };
        }
        20 => {
            exti.imr1.modify(|_, w| w.mr20().unmasked());
            exti.rtsr1.modify(|_, w| w.tr20().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr20().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::RTC_WKUP) };
        }
        // 21 => {
        //     exti.imr1.modify(|_, w| w.mr21().unmasked());
        //     exti.rtsr1.modify(|_, w| w.tr21().bit(rise_trigger));
        //     exti.ftsr1.modify(|_, w| w.tr21().bit(!rise_trigger));
        //     unsafe { NVIC::unmask(interrupt::todocomparator 1) };
        // }
        // 22 => {
        //     exti.imr1.modify(|_, w| w.mr22().unmasked());
        //     exti.rtsr1.modify(|_, w| w.tr22().bit(rise_trigger));
        //     exti.ftsr1.modify(|_, w| w.tr22().bit(!rise_trigger));
        //     unsafe { NVIC::unmask(interrupt::totocomparator2) };
        // }
        23 => {
            exti.imr1.modify(|_, w| w.mr23().unmasked());
            // todo: trigger edges tr23-28 and 34-35?
            // exti.rtsr1.modify(|_, w| w.tr23().bit(rise_trigger));
            // exti.ftsr1.modify(|_, w| w.tr23().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::I2C1_EV_EXTI23) };
        }
        24 => {
            exti.imr1.modify(|_, w| w.mr24().unmasked());
            // exti.rtsr1.modify(|_, w| w.tr24().bit(rise_trigger));
            // exti.ftsr1.modify(|_, w| w.tr24().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::I2C2_EV_EXTI24) };
        }
        25 => {
            exti.imr1.modify(|_, w| w.mr25().unmasked());
            // exti.rtsr1.modify(|_, w| w.tr25().bit(rise_trigger));
            // exti.ftsr1.modify(|_, w| w.tr25().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::USART1_EXTI25) };
        }
        26 => {
            exti.imr1.modify(|_, w| w.mr26().unmasked());
            // exti.rtsr1.modify(|_, w| w.tr26().bit(rise_trigger));
            // exti.ftsr1.modify(|_, w| w.tr26().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::USART2_EXTI26) };
        }
        27 => {
            exti.imr1.modify(|_, w| w.mr27().unmasked());
            // exti.rtsr1.modify(|_, w| w.tr27().bit(rise_trigger));
            // exti.ftsr1.modify(|_, w| w.tr27().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::I2C3_EV) };
        }
        28 => {
            exti.imr1.modify(|_, w| w.mr28().unmasked());
            // exti.rtsr1.modify(|_, w| w.tr28().bit(rise_trigger));
            // exti.ftsr1.modify(|_, w| w.tr28().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::USART3_EXTI28) };
        }
        // 29 => {
        //     exti.imr1.modify(|_, w| w.mr29().unmasked());
        //     exti.rtsr1.modify(|_, w| w.tr29().bit(rise_trigger));
        //     exti.ftsr1.modify(|_, w| w.tr29().bit(!rise_trigger));
        //     unsafe { NVIC::unmask(interrupt::todocomparator 3) };
        // }
        // 30 => {
        //     exti.imr1.modify(|_, w| w.mr30().unmasked());
        //     exti.rtsr1.modify(|_, w| w.tr30().bit(rise_trigger));
        //     exti.ftsr1.modify(|_, w| w.tr30().bit(!rise_trigger));
        //     unsafe { NVIC::unmask(interrupt::todocomparator4) };
        // }
        // 31 => {
        //     exti.imr1.modify(|_, w| w.mr31().unmasked());
        //     exti.rtsr1.modify(|_, w| w.tr31().bit(rise_trigger));
        //     exti.ftsr1.modify(|_, w| w.tr31().bit(!rise_trigger));
        //     unsafe { NVIC::unmask(interrupt::todocomparator5) };
        // }
        // 32 => {
        //     exti.imr2.modify(|_, w| w.mr32().unmasked());
        //     exti.rtsr2.modify(|_, w| w.tr32().bit(rise_trigger));
        //     exti.ftsr2.modify(|_, w| w.tr32().bit(!rise_trigger));
        //     unsafe { NVIC::unmask(interrupt::todocomparator6) };
        // }
        // 33 => {
        //     exti.imr2.modify(|_, w| w.mr33().unmasked());
        //     exti.rtsr2.modify(|_, w| w.tr33().bit(rise_trigger));
        //     exti.ftsr2.modify(|_, w| w.tr33().bit(!rise_trigger));
        //     unsafe { NVIC::unmask(interrupt::todocomparator7) };
        // }
        34 => {
            exti.imr2.modify(|_, w| w.mr34().unmasked());
            // exti.rtsr1.modify(|_, w| w.tr34().bit(rise_trigger));
            // exti.ftsr1.modify(|_, w| w.tr34().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::UART4_EXTI34) };
        }
        35 => {
            exti.imr2.modify(|_, w| w.mr35().unmasked());
            // exti.rtsr2.modify(|_, w| w.tr35().bit(rise_trigger));
            // exti.ftsr2.modify(|_, w| w.tr35().bit(!rise_trigger));
            unsafe { NVIC::unmask(interrupt::UART5_EXTI35) };
        }

        _ => panic!("Interrupt line must be between 0 and 35."),
    }
}

// Setup functions to bind a push button that an interrupt line

/// Configure a GPIO to work with an interrupt line.
/// The `line` parameter corresponds to both the interrupt line, and the
/// GPIO number. Eg, `3` means `line3`, and `px3`.
/// Reference section 12.1.3 of the datasheet for a breakdown by CR
/// and line.
pub fn setup_gpio(syscfg: &mut SYSCFG, gpio_reg: GpioReg, line: u8) {
    // Select this GPIO pin as source input for EXTI line external interrupt

    match line {
        0 => syscfg
            .exticr1
            .modify(|_, w| unsafe { w.exti0().bits(gpio_reg.cr_val()) }),
        1 => syscfg
            .exticr1
            .modify(|_, w| unsafe { w.exti1().bits(gpio_reg.cr_val()) }),
        2 => syscfg
            .exticr1
            .modify(|_, w| unsafe { w.exti2().bits(gpio_reg.cr_val()) }),
        3 => syscfg
            .exticr1
            .modify(|_, w| unsafe { w.exti3().bits(gpio_reg.cr_val()) }),
        4 => syscfg
            .exticr2
            .modify(|_, w| unsafe { w.exti4().bits(gpio_reg.cr_val()) }),
        5 => syscfg
            .exticr2
            .modify(|_, w| unsafe { w.exti5().bits(gpio_reg.cr_val()) }),
        6 => syscfg
            .exticr2
            .modify(|_, w| unsafe { w.exti6().bits(gpio_reg.cr_val()) }),
        7 => syscfg
            .exticr2
            .modify(|_, w| unsafe { w.exti7().bits(gpio_reg.cr_val()) }),
        8 => syscfg
            .exticr3
            .modify(|_, w| unsafe { w.exti8().bits(gpio_reg.cr_val()) }),
        9 => syscfg
            .exticr3
            .modify(|_, w| unsafe { w.exti9().bits(gpio_reg.cr_val()) }),
        10 => syscfg
            .exticr3
            .modify(|_, w| unsafe { w.exti10().bits(gpio_reg.cr_val()) }),
        11 => syscfg
            .exticr3
            .modify(|_, w| unsafe { w.exti11().bits(gpio_reg.cr_val()) }),
        12 => syscfg
            .exticr4
            .modify(|_, w| unsafe { w.exti12().bits(gpio_reg.cr_val()) }),
        13 => syscfg
            .exticr4
            .modify(|_, w| unsafe { w.exti13().bits(gpio_reg.cr_val()) }),
        14 => syscfg
            .exticr4
            .modify(|_, w| unsafe { w.exti14().bits(gpio_reg.cr_val()) }),
        15 => syscfg
            .exticr4
            .modify(|_, w| unsafe { w.exti15().bits(gpio_reg.cr_val()) }),
        _ => panic!("Can only setup GPIO interrupts on lines 0 - 15."),
    }
}

/// Process is from the STM32F3 Ref Man, Section 27.5
pub fn setup_rtc_wakeup(syscfg: &mut SYSCFG, exti: &mut EXTI, rtc: &mut RTC) {
    // todo RTC wakeup: Pc13 ??
    // Configure and enable the EXTI line corresponding to the Wakeup timer even in
    // interrupt mode and select the rising edge sensitivity.
    // Configure and enable the RTC_WKUP IRQ channel in the NVIC.
    // (Both of those steps above are handled by `setup_line`.)
    setup_line(exti, 20, Edge::Rising);

    // Configure the RTC to detect the RTC Wakeup timer event.
    // event flag: WUTF. Enable control bit: WUTIE
    // rtc.isr.modify(|_, w| w.wutf().bit(true));  // todo: Is this what we want?

    // Enable the wakeup timer
    rtc.cr.modify(|_, w| w.wutie().bit(true)); // todo: Is this what we want?
                                               // Enable the wakeup timer interrupt
                                               // Ref man section 27.6.6: When the wakeup timer is enabled (WUTE set to 1), the WUTF flag
                                               // is set every (WUT[15:0]
                                               // + 1) ck_wut cycles. The ck_wut period is selected through WUCKSEL[2:0] bits of the
                                               // RTC_CR register
                                               // When WUCKSEL[2] = 1, the wakeup timer becomes 17-bits and WUCKSEL[1] effectively
                                               // becomes WUT[16] the most-significant bit to be reloaded into the timer.
                                               // The first assertion of WUTF occurs (WUT+1) ck_wut cycles after WUTE is set. Setting
                                               // WUT[15:0] to 0x0000 with WUCKSEL[2:0] =011 (RTCCLK/2) is forbidden.
    rtc.cr.modify(|_, w| w.wute().bit(true));
    // Wakeup clock selection
    rtc.cr.modify(|_, w| unsafe { w.wcksel().bits(0x000) }); // pg 295 todo: Is this what we want?
}

// From the Ref manual, page 296:
// The remaining lines are connected as follows:
// • EXTI line 16 is connected to the PVD output
// • EXTI line 17 is connected to the RTC Alarm event
// • EXTI line 18 is connected to USB Device FS wakeup event (STM32F303xB/C/D/E,
// STM32F358xC and STM32F398xE devices)
// • EXTI line 19 is connected to RTC tamper and Timestamps
// • EXTI line 20 is connected to RTC wakeup timer
// • EXTI line 21 is connected to Comparator 1 output (STM32F303xB/C/D/E,
// STM32F358xC and STM32F398xE devices)
// • EXTI line 22 is connected to Comparator 2 output
// • EXTI line 23 is connected to I2C1 wakeup
// • EXTI line 24 is connected to I2C2 wakeup (STM32F303xB/C/D/E, STM32F358xC and
// STM32F398xE devices)
// • EXTI line 25 is connected to USART1 wakeup
// • EXTI line 26 is connected to USART2 wakeup (STM32F303xB/C/D/E, STM32F358xC
// and STM32F398xE devices)
// • EXTI line 27 is connected to I2C3 wakeup)(STM32F303xD/E and STM32F398
// devices)
// • EXTI line 28 is connected to USART3 wakeup (STM32F303xB/C/D/E, STM32F358xC
// and STM32F398xE devices)
// • EXTI line 29 is connected to Comparator 3 output (STM32F303xB/C/D/E,
// STM32F358xC and STM32F398xE devices)
// • EXTI line 30 is connected to Comparator 4 output
// • EXTI line 31 is connected to Comparator 5 output (STM32F303xB/C/D/E,
// STM32F358xC and STM32F398xE devices)
// • EXTI line 32 is connected to Comparator 6 output
// • EXTI line 33 is connected to Comparator 7 output (STM32F303xB/C/D/E,
// STM32F358xC and STM32F398xE devices)
// • EXTI line 34 is connected to UART4 wakeup (STM32F303xB/C/D/E, STM32F358xC
// and STM32F398xE devices)
// • EXTI line 35 is connected to UART5 wakeup (STM32F303xB/C/D/E, STM32F358xC
// and STM32F398xE devices)
// Note: EXTI lines 23, 24, 25, 26, 27, 28, 34 and 35 are internal
