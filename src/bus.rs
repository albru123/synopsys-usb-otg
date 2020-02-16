use crate::endpoint_allocator::SynopsysEndpointAllocator;
use crate::endpoint_memory::EndpointBufferState;
use crate::endpoint_trait::*;
use crate::ral::{modify_reg, otg_device, otg_global, otg_pwrclk, read_reg, write_reg};
use crate::target::interrupt::{self, CriticalSection, Mutex};
use crate::target::UsbRegisters;
use crate::UsbPeripheral;
use core::cmp;
use usb_device::allocator::UsbAllocator;
use usb_device::bus::PollResult;
use usb_device::endpoint::{Endpoint, EndpointAddress};
use usb_device::UsbDirection;

/// USB peripheral driver for STM32 microcontrollers.
pub struct UsbBus<USB> {
    peripheral: USB,
    regs: Mutex<UsbRegisters<USB>>,
    // endpoints_in: [EndpointIn; 4],
    // endpoints_out: [EndpointOut; 4],
    endpoint_allocator: SynopsysEndpointAllocator,
}

impl<USB: UsbPeripheral> UsbBus<USB> {
    /// Constructs a new USB peripheral driver.
    pub fn new(peripheral: USB, ep_memory: &'static mut [u32]) -> UsbAllocator<Self> {
        let bus = UsbBus {
            peripheral,
            regs: Mutex::new(UsbRegisters::new()),
            endpoint_allocator: SynopsysEndpointAllocator::new(ep_memory),
        };

        UsbAllocator::new(bus)
    }

    pub fn free(self) -> USB {
        self.peripheral
    }

    pub fn configure_all(&self, cs: &CriticalSection) {
        let regs = self.regs.borrow(cs);

        // Rx FIFO
        let rx_fifo_size = if USB::HIGH_SPEED {
            self.endpoint_allocator
                .memory_allocator
                .total_rx_buffer_size_words() as u32
                + 30
        } else {
            self.endpoint_allocator
                .memory_allocator
                .total_rx_buffer_size_words() as u32
                + 20
        };
        write_reg!(otg_global, regs.global, GRXFSIZ, rx_fifo_size);
        let mut fifo_top = rx_fifo_size;

        // Tx FIFO #0
        let fifo_size = cmp::max(
            self.endpoint_allocator.endpoints_in[0].fifo_size_words(),
            16,
        );

        #[cfg(feature = "fs")]
        write_reg!(
            otg_global,
            regs.global,
            DIEPTXF0,
            TX0FD: fifo_size,
            TX0FSA: fifo_top
        );
        #[cfg(feature = "hs")]
        write_reg!(
            otg_global,
            regs.global,
            GNPTXFSIZ,
            TX0FD: fifo_size,
            TX0FSA: fifo_top
        );

        fifo_top += fifo_size;

        // Tx FIFO #1
        let fifo_size = cmp::max(
            self.endpoint_allocator.endpoints_in[1].fifo_size_words(),
            16,
        );
        write_reg!(
            otg_global,
            regs.global,
            DIEPTXF1,
            INEPTXFD: fifo_size,
            INEPTXSA: fifo_top
        );
        fifo_top += fifo_size;

        // Tx FIFO #2
        let fifo_size = cmp::max(
            self.endpoint_allocator.endpoints_in[2].fifo_size_words(),
            16,
        );
        write_reg!(
            otg_global,
            regs.global,
            DIEPTXF2,
            INEPTXFD: fifo_size,
            INEPTXSA: fifo_top
        );
        fifo_top += fifo_size;

        // Tx FIFO #3
        let fifo_size = cmp::max(
            self.endpoint_allocator.endpoints_in[3].fifo_size_words(),
            16,
        );
        write_reg!(
            otg_global,
            regs.global,
            DIEPTXF3,
            INEPTXFD: fifo_size,
            INEPTXSA: fifo_top
        );
        fifo_top += fifo_size;

        assert!(fifo_top <= crate::ral::otg_fifo::FIFO_DEPTH_WORDS);

        // Flush Rx & Tx FIFOs
        modify_reg!(otg_global, regs.global, GRSTCTL, RXFFLSH: 1, TXFFLSH: 1, TXFNUM: 0x10);
        while read_reg!(otg_global, regs.global, GRSTCTL, RXFFLSH, TXFFLSH) != (0, 0) {}

        for ep in &self.endpoint_allocator.endpoints_in {
            if ep.is_initialized() {
                // enabling EP TX interrupt
                modify_reg!(otg_device, regs.device, DAINTMSK, |v| v
                    | (0x0001 << ep.address().number()));

                ep.enable();
            }
        }

        for ep in &self.endpoint_allocator.endpoints_out {
            if ep.is_initialized() {
                if ep.address().number() == 0 {
                    // enabling RX interrupt from EP0
                    modify_reg!(otg_device, regs.device, DAINTMSK, |v| v | 0x00010000);
                }

                ep.enable();
            }
        }
    }

    pub fn deconfigure_all(&self, cs: &CriticalSection) {
        let regs = self.regs.borrow(cs);

        // disable interrupts
        modify_reg!(otg_device, regs.device, DAINTMSK, IEPM: 0, OEPM: 0);

        for ep in &self.endpoint_allocator.endpoints_in {
            ep.disable();
        }

        for ep in &self.endpoint_allocator.endpoints_out {
            ep.disable();
        }
    }
}

impl<USB: UsbPeripheral> usb_device::bus::UsbBus for UsbBus<USB> {
    type EndpointAllocator = SynopsysEndpointAllocator;
    type EndpointOut = SynopsysEndpoint;
    type EndpointIn = SynopsysEndpoint;

    fn create_allocator(&mut self) -> Self::EndpointAllocator {
        self.endpoint_allocator
    }

    fn enable(&mut self) {
        // Enable USB_OTG in RCC
        USB::enable();

        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);

            // Wait for AHB ready
            while read_reg!(otg_global, regs.global, GRSTCTL, AHBIDL) == 0 {}

            // Configure OTG as device
            #[cfg(feature = "fs")]
            modify_reg!(otg_global, regs.global, GUSBCFG,
                SRPCAP: 0, // SRP capability is not enabled
                TRDT: 0x6, // ??? USB turnaround time
                FDMOD: 1 // Force device mode
            );
            #[cfg(feature = "hs")]
            modify_reg!(otg_global, regs.global, GUSBCFG,
                SRPCAP: 0, // SRP capability is not enabled
                TRDT: 0x9, // ??? USB turnaround time
                TOCAL: 0x1,
                FDMOD: 1, // Force device mode
                PHYSEL: 1
            );

            // Configuring Vbus sense and SOF output
            //write_reg!(otg_global, regs.global, GCCFG, VBUSBSEN: 1);
            write_reg!(otg_global, regs.global, GCCFG, 1 << 21); // set NOVBUSSENS

            // Enable PHY clock
            write_reg!(otg_pwrclk, regs.pwrclk, PCGCCTL, 0);

            // Soft disconnect device
            modify_reg!(otg_device, regs.device, DCTL, SDIS: 1);

            // Setup USB FS speed [and frame interval]
            modify_reg!(otg_device, regs.device, DCFG,
                DSPD: 0b11 // Device speed: Full speed
            );

            // unmask EP interrupts
            write_reg!(otg_device, regs.device, DIEPMSK, XFRCM: 1);

            // unmask core interrupts
            write_reg!(otg_global, regs.global, GINTMSK,
                USBRST: 1, ENUMDNEM: 1,
                USBSUSPM: 1, WUIM: 1,
                IEPINT: 1, RXFLVLM: 1
            );

            // clear pending interrupts
            write_reg!(otg_global, regs.global, GINTSTS, 0xffffffff);

            // unmask global interrupt
            modify_reg!(otg_global, regs.global, GAHBCFG, GINT: 1);

            // connect(true)
            modify_reg!(otg_global, regs.global, GCCFG, PWRDWN: 1);
            modify_reg!(otg_device, regs.device, DCTL, SDIS: 0);
        });
    }

    fn reset(&mut self) {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);

            self.configure_all(cs);

            modify_reg!(otg_device, regs.device, DCFG, DAD: 0);
        });
    }

    fn set_device_address(&mut self, addr: u8) {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);

            modify_reg!(otg_device, regs.device, DCFG, DAD: addr as u32);
        });
    }

    fn set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        if ep_addr.number() >= 4 {
            return;
        }

        match ep_addr.direction() {
            UsbDirection::In => {
                self.endpoint_allocator.endpoints_in[ep_addr.number() as usize].set_stalled(stalled)
            }
            UsbDirection::Out => self.endpoint_allocator.endpoints_out[ep_addr.number() as usize]
                .set_stalled(stalled),
        }
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        if ep_addr.number() >= 4 {
            return true;
        }

        match ep_addr.direction() {
            UsbDirection::In => {
                self.endpoint_allocator.endpoints_in[ep_addr.number() as usize].is_stalled()
            }
            UsbDirection::Out => {
                self.endpoint_allocator.endpoints_out[ep_addr.number() as usize].is_stalled()
            }
        }
    }

    fn suspend(&mut self) {
        // Nothing to do here?
    }

    fn resume(&mut self) {
        // Nothing to do here?
    }

    fn poll(&mut self) -> PollResult {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);

            let (wakeup, suspend, enum_done, reset, iep, rxflvl) = read_reg!(
                otg_global,
                regs.global,
                GINTSTS,
                WKUPINT,
                USBSUSP,
                ENUMDNE,
                USBRST,
                IEPINT,
                RXFLVL
            );

            if reset != 0 {
                write_reg!(otg_global, regs.global, GINTSTS, USBRST: 1);

                self.deconfigure_all(cs);

                // Flush RX
                modify_reg!(otg_global, regs.global, GRSTCTL, RXFFLSH: 1);
                while read_reg!(otg_global, regs.global, GRSTCTL, RXFFLSH) == 1 {}
            }

            if enum_done != 0 {
                write_reg!(otg_global, regs.global, GINTSTS, ENUMDNE: 1);

                PollResult::Reset
            } else if wakeup != 0 {
                // Clear the interrupt
                write_reg!(otg_global, regs.global, GINTSTS, WKUPINT: 1);

                PollResult::Resume
            } else if suspend != 0 {
                write_reg!(otg_global, regs.global, GINTSTS, USBSUSP: 1);

                PollResult::Suspend
            } else {
                let mut ep_out = 0;
                let mut ep_in_complete = 0;
                let mut ep_setup = 0;

                use crate::ral::{endpoint_in, endpoint_out};

                // RXFLVL & IEPINT flags are read-only, there is no need to clear them
                if rxflvl != 0 {
                    let (epnum, data_size, status) =
                        read_reg!(otg_global, regs.global, GRXSTSR, EPNUM, BCNT, PKTSTS);
                    match status {
                        0x02 => {
                            // OUT received
                            ep_out |= 1 << epnum;
                        }
                        0x06 => {
                            // SETUP received
                            // flushing TX if something stuck in control endpoint
                            let ep = endpoint_in::instance(epnum as u8);
                            if read_reg!(endpoint_in, ep, DIEPTSIZ, PKTCNT) != 0 {
                                modify_reg!(otg_global, regs.global, GRSTCTL, TXFNUM: epnum, TXFFLSH: 1);
                                while read_reg!(otg_global, regs.global, GRSTCTL, TXFFLSH) == 1 {}
                            }
                            ep_setup |= 1 << epnum;
                        }
                        0x03 | 0x04 => {
                            // OUT completed | SETUP completed
                            let ep = endpoint_out::instance(epnum as u8);
                            modify_reg!(endpoint_out, ep, DOEPCTL, CNAK: 1, EPENA: 1);
                            read_reg!(otg_global, regs.global, GRXSTSP); // pop GRXSTSP
                        }
                        _ => {
                            read_reg!(otg_global, regs.global, GRXSTSP); // pop GRXSTSP
                        }
                    }

                    if status == 0x02 || status == 0x06 {
                        let ep = &self.endpoint_allocator.endpoints_out[epnum as usize];

                        let mut buffer = ep.buffer.unwrap().borrow(cs).borrow_mut();
                        if buffer.state() == EndpointBufferState::Empty {
                            read_reg!(otg_global, regs.global, GRXSTSP); // pop GRXSTSP

                            let is_setup = status == 0x06;
                            buffer.fill_from_fifo(data_size as u16, is_setup).ok();
                        }
                    }
                }

                if iep != 0 {
                    for ep in &self.endpoint_allocator.endpoints_in {
                        if ep.is_initialized() {
                            let ep_regs = endpoint_in::instance(ep.address().number());
                            if read_reg!(endpoint_in, ep_regs, DIEPINT, XFRC) != 0 {
                                write_reg!(endpoint_in, ep_regs, DIEPINT, XFRC: 1);
                                ep_in_complete |= 1 << ep.address().number();
                            }
                        }
                    }
                }

                for ep in &self.endpoint_allocator.endpoints_out {
                    if ep.is_initialized() {
                        match ep.buffer_state() {
                            EndpointBufferState::DataOut => {
                                ep_out |= 1 << ep.address().number();
                            }
                            EndpointBufferState::DataSetup => {
                                ep_setup |= 1 << ep.address().number();
                            }
                            EndpointBufferState::Empty => {}
                        }
                    }
                }

                if (ep_in_complete | ep_out | ep_setup) != 0 {
                    PollResult::Data {
                        ep_out,
                        ep_in_complete,
                        ep_setup,
                    }
                } else {
                    PollResult::None
                }
            }
        })
    }

    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = true;
}
