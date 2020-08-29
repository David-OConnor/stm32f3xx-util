//! Configure interrupts for the stm32f3xx.
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

const LFE_FREQ: u32 = 32_768;

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

    // todo: `enr` is private, but perhaps that pointer math will do the same.
    // apb1.enr().modify(|_, w| w.pwren().set_bit());
    // unsafe { (*RTC::ptr()).enr().modify(|_, w| w.pren().set_bit()) };
    // unsafe { *RCC::ptr().apb1enr().modify(|_, w| w.pwren().enabled()) }
    // todo: Put back a pwren modifier, either normal code with modded
    // todo hal lib to make enr pub, or
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
                unsafe { (*EXTI::ptr()).pr1.modify(|_, w| w.pr1().bit(true)) };
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
pub fn setup_rtc_wakeup(syscfg: &mut SYSCFG, exti: &mut EXTI, rtc: &mut RTC, time_s: u32, time_us: u32) {
    // todo: Split into initial config, and timer start fns?
    // todo: Or a fn to set up wakeup, and another to set up the interrupt.

     // 27.3.6 Periodic auto-wakeup
    // The periodic wakeup flag is generated by a 16-bit programmable auto-reload down-counter.
    // The wakeup timer range can be extended to 17 bits.
    // The wakeup function is enabled through the WUTE bit in the RTC_CR register.
    // The wakeup timer clock input can be:
    // • RTC clock (RTCCLK) divided by 2, 4, 8, or 16.
    // When RTCCLK is LSE(32.768kHz), this allows to configure the wakeup interrupt period
    // from 122 µs to 32 s, with a resolution down to 61 µs.
    // • ck_spre (usually 1 Hz internal clock)
    // When ck_spre frequency is 1Hz, this allows to achieve a wakeup time from 1 s to
    // around 36 hours with one-second resolution. This large programmable time range is
    // divided in 2 parts:
    // – from 1s to 18 hours when WUCKSEL [2:1] = 10
    // – and from around 18h to 36h when WUCKSEL[2:1] = 11. In this last case 216 is
    // added to the 16-bit counter current value.When the initialization sequence is
    // complete (see Programming the wakeup timer on page 781), the timer starts
    // counting down.When the wakeup function is enabled, the down-counting remains
    // active in low-power modes. In addition, when it reaches 0, the WUTF flag is set in
    // Real-time clock (RTC) RM0316
    // 780/1141 DocID022558 Rev 8
    // the RTC_ISR register, and the wakeup counter is automatically reloaded with its
    // reload value (RTC_WUTR register value).
    // The WUTF flag must then be cleared by software.
    // When the periodic wakeup interrupt is enabled by setting the WUTIE bit in the RTC_CR2
    // register, it can exit the device from low-power modes.
    // The periodic wakeup flag can be routed to the RTC_ALARM output provided it has been
    // enabled through bits OSEL[1:0] of RTC_CR register. RTC_ALARM output polarity can be
    // configured through the POL bit in the RTC_CR register.
    // System reset, as well as low-power modes (Sleep, Stop and Standby) have no influence on
    // the wakeup timer

    // See also: ST AN2759, Table 11

    // Ref man, section 27.5: RTC Interrups:
    // Configure and enable the EXTI line corresponding to the Wakeup timer even in
    // interrupt mode and select the rising edge sensitivity.
    // Configure and enable the RTC_WKUP IRQ channel in the NVIC.
    // (Both of those steps above are handled by `setup_line`.)
    setup_line(exti, 20, Edge::Rising);

    // using-thehard-ware-realtime-clock, section 2.13:
    // To protect the RTC registers against possible unintentional write accesses after reset, the
    // RTC registers are initially, after a backup domain reset, locked. They must be unlocked to
    // update the current calendar time and date.
    // The write-access to the RTC registers is enabled by writing a key in the write protection
    // register (RTC_WPR).
    // The following sequence is required to unlock the write protection of the RTC register:
    // 1. Write 0xCA into the RTC_WPR register.
    // 2. Write 0x53 into the RTC_WPR register.
    // Any write access to the RTC_WPR register different from the above described write
    // sequence activates the write-protection mechanism for the RTC registers.
    rtc.wpr.write(|w| unsafe {w.bits(0xCA) });
    rtc.wpr.write(|w| unsafe {w.bits(0x53) });

    // Ref man, page 781:
    // Programming the wakeup timer
    // The following sequence is required to configure or change the wakeup timer auto-reload
    // value (WUT[15:0] in RTC_WUTR):

    // 1. Clear WUTE in RTC_CR to disable the wakeup timer.
    rtc.cr.modify(|_, w| w.wute().clear_bit());

    // 2. Poll WUTWF until it is set in RTC_ISR to make sure the access to wakeup auto-reload
    // counter and to WUCKSEL[2:0] bits is allowed. It takes around 2 RTCCLK clock cycles
    // (due to clock synchronization).
    while !rtc.isr.read().wutwf().bit_is_set() {}

    // 3. Program the wakeup auto-reload value WUT[15:0]...
    // todo: May need to pass in which oscillator you're using, or its freq. Maybe an enum.
    // todo: This could be a big number. What if it overflows?
    // todo: And it needs to be u16, whcih will certainly cause it to overflow.
    // let sleep_for_cycles = LFE_FREQ * time_ms / 1000;
    let sleep_for_cycles = 1000;
    rtc.wutr.modify(|_, w| unsafe { w.wut().bits(sleep_for_cycles) });

    // and the wakeup clock selection
    // (WUCKSEL[2:0] bits in RTC_CR).
    // The wakeup timer restarts down-counting. The WUTWF bit is cleared up to 2 RTCCLK
    // clock cycles after WUTE is cleared, due to clock synchronization.
    // See pg 795 for info on `WUCKSEL[2:0].
    // todo: Is this what we want? 000 sets RTC/16.
    // RTC clock (RTCCLK) divided by 2, 4, 8, or 16.
    // When RTCCLK is LSE(32.768kHz), this allows to configure the wakeup interrupt period
    // from 122 µs to 32 s, with a resolution down to 61 µs.

    // See Using-the-hardware-realtime-clock-rtc document, section 2.4.2 for
    // details on WUCKSEL.
    // Configuration 1: WUCKSEL[2:0] = 0b0xx for short wakeup periods
    // (see Periodic timebase/wakeup configuration for clock configuration 1)
    // • Configuration 2: WUCKSEL[2:0] = 0b10x for medium wakeup periods
    // (see Periodic timebase/wakeup configuration for clock configuration 2)
    // • Configuration 3: WUCKSEL[2:0] = 0b11x for long wakeup periods
    // (see Periodic timebase/wakeup configuration for clock configuration 3)

    // Config 1:
    // The minimum timebase/wakeup period is (0x0001 + 1) x 61.035 µs = 122.07 µs.
    // The maximum timebase/wakeup period is (0xFFFF+ 1) x 488.28 µs = 32 s.
    // Config 2:
    // The minimum timebase/wakeup period is (0x0000 + 1) x 1 s = 1 s.
    // The maximum timebase/wakeup period is (0xFFFF+ 1) x 1 s = 65536 s (18 hours).
    // Config 3:
    // The minimum timebase/wakeup period is (0x10000 + 1) x 1 s = 65537 s (18 hours + 1 s)
    // The maximum timebase/wakeup period is (0x1FFFF+ 1) x 1 s = 131072 s (36 hours).
    // todo: Pick a WUCKSEL and WUT etc based on the requested sleep time.
    rtc.cr.modify(|_, w| unsafe { w.wcksel().bits(0b000) });

    // Set WUTE in RTC_CR to enable the timer again.
    rtc.cr.modify(|_, w| w.wute().bit(true));

    // Write 0xFF into the RTC_WPR register
    rtc.wpr.write(|w| unsafe {w.bits(0xFF) });

    // Configure the RTC to detect the RTC Wakeup timer event.
    // event flag: WUTF. Enable control bit: WUTIE
    // rtc.isr.modify(|_, w| w.wutf().bit(true));  // todo: Is this what we want?

    // todo: Cr2 for wutie?
    // Enable the wakeup timer interrupt.
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

    // Clear wakeup timer flag (?)

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
