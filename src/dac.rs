//! Configure the internal DAC on the stm32f3xx.

use stm32f3xx_hal::stm32;

#[derive(Clone, Copy, Debug)]
pub enum DacId {
    One,
    Two
}

#[derive(Clone, Copy, Debug)]
pub enum DacBits {
    EightR,
    TwelveL,
    TwelveR,
}

pub struct Dac {
    regs: stm32::DAC,
    id: DacId,
    bits: DacBits,
}

impl Dac {
    /// Create a new DAC instances
    pub fn new(regs: stm32::DAC, id: DacId, bits: DacBits) -> Self {
        Self { regs, id, bits }
    }

    /// Enable the DAC.
    pub fn enable(&mut self) {
        match self.id {
            DacId::One => self.regs.cr.modify(|_, w| w.en1().set_bit()),
            DacId::Two => self.regs.cr.modify(|_, w| w.en2().set_bit()),
        }
    }

    /// Set the DAC voltage. `v` is in Volts.
    pub fn set_voltage(&mut self, volts: f32) {
        let max_volts = 3.3;

        // todo: Dac1 vs 2
        match self.bits {
            DacBits::EightR => {
                let val = ((volts / max_volts) * 255.) as u32;
                self.regs.dhr8r1.modify(|_, w| unsafe { w.bits(val)})
            }
            DacBits::TwelveL => {
                let val = ((volts / max_volts) * 4_095.) as u32;
                self.regs.dhr12l1.modify(|_, w| unsafe { w.bits(val)})
            }
            DacBits::TwelveR => {
                let val = ((volts / max_volts) * 4_095.) as u32;
                self.regs.dhr12r1.modify(|_, w| unsafe { w.bits(val)})
            }
        }

    }

    /// Independent trigger with single LFSR generation
    pub fn trigger_lfsr(&mut self) {

    }

    /// Independent trigger with single triangle generation
    pub fn trigger_triangle(&mut self) {

    }
}