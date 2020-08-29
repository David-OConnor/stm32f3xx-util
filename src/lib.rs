#![no_std]

// underscore to not interfere with `[#interrupt] in generated fn.
pub mod dac;
pub mod interrupt_;
pub mod low_power;
