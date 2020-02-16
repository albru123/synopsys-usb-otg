//! USB peripheral driver for Synopsys USB OTG peripherals.

#![no_std]

#[cfg(all(feature = "fs", feature = "hs"))]
compile_error!("choose only one USB mode");

#[cfg(not(any(feature = "fs", feature ="hs")))]
compile_error!("select USB mode feature (fs/hs)");

mod endpoint;
mod endpoint_memory;
mod endpoint_allocator;
mod endpoint_trait;

mod target;

/// USB peripheral driver.
pub mod bus;

pub use crate::bus::UsbBus;

mod ral;

/// A trait for device-specific USB peripherals. Implement this to add support for a new hardware
/// platform. Peripherals that have this trait must have the same register block as STM32 USB OTG
/// peripherals.
pub unsafe trait UsbPeripheral: Send + Sync {
    /// Pointer to the register block
    const REGISTERS: *const ();

    /// true for High Speed variants of the peripheral, false for Full Speed
    const HIGH_SPEED: bool;

    /// FIFO size in 32-bit words
    const FIFO_DEPTH_WORDS: usize;

    /// Enables USB device on its peripheral bus
    fn enable();
}
