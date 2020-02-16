use crate::endpoint_memory::{EndpointBuffer, EndpointBufferState};
use crate::ral::{endpoint0_out, endpoint_in, endpoint_out, modify_reg, read_reg, write_reg};
use crate::target::fifo_write;
use crate::target::interrupt::{self, Mutex};

use core::cell::RefCell;
use usb_device::allocator::EndpointConfig;
use usb_device::endpoint::*;
use usb_device::{Result, UsbDirection, UsbError};

/// Arbitrates access to the endpoint-specific registers and packet buffer memory.
pub struct SynopsysEndpoint {
    pub address: EndpointAddress,
    pub configuration: Option<SynopsysEndpointConfiguration>,
    pub buffer: Option<Mutex<RefCell<EndpointBuffer>>>,
}

pub struct SynopsysEndpointConfiguration {
    pub ep_type: EndpointType,
    pub max_packet_size: u16,
    pub interval: u8,
    pub audio_streaming_extension: Option<AudioStreamingExtension>,
}

impl SynopsysEndpoint {
    pub fn new(address: EndpointAddress) -> SynopsysEndpoint {
        SynopsysEndpoint {
            address,
            configuration: None,
            buffer: None
        }
    }

    pub fn initialize(&mut self, direction: UsbDirection, config: &EndpointConfig, buffer: Option<EndpointBuffer>) {
        let audio_streaming_extension = if config.is_audio_streaming {
            Some(AudioStreamingExtension {
                synchronization_address: None,
            })
        } else {
            None
        };

        self.configuration = Some(SynopsysEndpointConfiguration {
            ep_type: config.ep_type,
            max_packet_size: config.max_packet_size,
            interval: config.interval,
            audio_streaming_extension,
        });

        if direction == UsbDirection::Out && buffer.is_some() {
            interrupt::free(|cs| {
                self.buffer.unwrap().borrow(cs).replace(buffer.unwrap());
            });
        }
    }

    pub fn is_initialized(&self) -> bool {
        self.configuration.is_some()
    }

    pub fn buffer_state(&self) -> EndpointBufferState {
        interrupt::free(|cs| {
            self.buffer.unwrap().borrow(cs).borrow().state()
        })
    }

    pub fn fifo_size_words(&self) -> u32 {
        if self.is_initialized() {
            (self.max_packet_size() as u32 + 3) / 4
        } else {
            0
        }
    }
}

impl Endpoint for SynopsysEndpoint {
    /// Gets the descriptor information for this endpoint.
    fn descriptor(&self) -> &EndpointDescriptor {
        match self.configuration {
            Some(config) => &EndpointDescriptor {
                address: self.address,
                ep_type: config.ep_type,
                max_packet_size: config.max_packet_size,
                interval: config.interval,
                audio_streaming_extension: config.audio_streaming_extension,
            },
            None => &EndpointDescriptor {
                address: self.address,
                ep_type: EndpointType::Control,
                max_packet_size: 0,
                interval: 0,
                audio_streaming_extension: None,
            },
        }
    }

    /// Gets the endpoint address.
    fn address(&self) -> EndpointAddress {
        self.address
    }

    /// Gets the endpoint transfer type.
    fn ep_type(&self) -> EndpointType {
        self.descriptor().ep_type
    }

    /// Gets the maximum packet size for the endpoint.
    fn max_packet_size(&self) -> u16 {
        self.descriptor().max_packet_size
    }

    /// Gets the poll interval for interrupt endpoints.
    fn interval(&self) -> u8 {
        self.descriptor().interval
    }

    /// Enables the endpoint with the specified configuration.
    fn enable(&mut self) {
        if self.address.number() == 0 {
            let mpsiz = match self.max_packet_size() {
                8 => 0b11,
                16 => 0b10,
                32 => 0b01,
                64 => 0b00,
                other => panic!("Unsupported EP0 size: {}", other),
            };

            if self.address.direction() == UsbDirection::In {
                let regs = endpoint_in::instance(self.address.number());

                write_reg!(endpoint_in, regs, DIEPCTL, MPSIZ: mpsiz as u32, SNAK: 1);

                write_reg!(endpoint_in, regs, DIEPTSIZ, PKTCNT: 0, XFRSIZ: self.max_packet_size() as u32);
            } else {
                let regs = endpoint0_out::instance();
                write_reg!(endpoint0_out, regs, DOEPTSIZ0, STUPCNT: 1, PKTCNT: 1, XFRSIZ: self.max_packet_size() as u32);
                modify_reg!(endpoint0_out, regs, DOEPCTL0, MPSIZ: mpsiz as u32, EPENA: 1, CNAK: 1);
            }
        } else {
            if self.address.direction() == UsbDirection::In {
                let regs = endpoint_in::instance(self.address.number());
                write_reg!(endpoint_in, regs, DIEPCTL,
                    SNAK: 1,
                    USBAEP: 1,
                    EPTYP: self.descriptor().ep_type as u32,
                    SD0PID_SEVNFRM: 1,
                    TXFNUM: self.address.number() as u32,
                    MPSIZ: self.max_packet_size() as u32
                );
            } else {
                let regs = endpoint_out::instance(self.address.number());
                write_reg!(endpoint_out, regs, DOEPCTL,
                    SD0PID_SEVNFRM: 1,
                    CNAK: 1,
                    EPENA: 1,
                    USBAEP: 1,
                    EPTYP: self.descriptor().ep_type as u32,
                    MPSIZ: self.max_packet_size() as u32
                );
            }
        }
    }

    /// Disables the endpoint.
    fn disable(&mut self) {
        if self.address.direction() == UsbDirection::In {
            let regs = endpoint_in::instance(self.address.number());

            // deactivating endpoint
            modify_reg!(endpoint_in, regs, DIEPCTL, USBAEP: 0);

            // TODO: flushing FIFO

            // disabling endpoint
            if read_reg!(endpoint_in, regs, DIEPCTL, EPENA) != 0 && self.address.number() != 0 {
                modify_reg!(endpoint_in, regs, DIEPCTL, EPDIS: 1)
            }

            // clean EP interrupts
            write_reg!(endpoint_in, regs, DIEPINT, 0xff);

        // TODO: deconfiguring TX FIFO
        } else {
            let regs = endpoint_out::instance(self.address.number());

            // deactivating endpoint
            modify_reg!(endpoint_out, regs, DOEPCTL, USBAEP: 0);

            // disabling endpoint
            if read_reg!(endpoint_out, regs, DOEPCTL, EPENA) != 0 && self.address.number() != 0 {
                modify_reg!(endpoint_out, regs, DOEPCTL, EPDIS: 1)
            }

            // clean EP interrupts
            write_reg!(endpoint_out, regs, DOEPINT, 0xff);
        }
    }

    /// Sets or clears the STALL condition for the endpoint. If the endpoint is an OUT endpoint, it
    /// will be prepared to receive data again.
    fn set_stalled(&mut self, stalled: bool) {
        if !self.is_initialized() {
            return;
        }

        interrupt::free(|_| {
            if self.is_stalled() == stalled {
                return;
            }

            if self.address.direction() == UsbDirection::In {
                let ep = endpoint_in::instance(self.address.number());
                modify_reg!(endpoint_in, ep, DIEPCTL, STALL: stalled as u32);
            } else {
                let ep = endpoint_out::instance(self.address.number());
                modify_reg!(endpoint_out, ep, DOEPCTL, STALL: stalled as u32);
            }
        })
    }

    /// Gets whether the STALL condition is set for an endpoint.
    fn is_stalled(&self) -> bool {
        let stall = if self.address.direction() == UsbDirection::In {
            let ep = endpoint_in::instance(self.address.number());
            read_reg!(endpoint_in, ep, DIEPCTL, STALL)
        } else {
            let ep = endpoint_out::instance(self.address.number());
            read_reg!(endpoint_out, ep, DOEPCTL, STALL)
        };

        stall != 0
    }
}

/// Handle for OUT endpoints.
impl EndpointOut for SynopsysEndpoint {
    /// Reads a single packet of data from the specified endpoint and returns the actual length of
    /// the packet. The buffer should be large enough to fit at least as many bytes as the
    /// `max_packet_size` specified when allocating the endpoint.
    ///
    /// # Errors
    ///
    /// Note: USB bus implementation errors are directly passed through, so be prepared to handle
    /// other errors as well.
    ///
    /// * [`WouldBlock`](crate::UsbError::WouldBlock) - There is no packet to be read. Note that
    ///   this is different from a received zero-length packet, which is valid and significant in
    ///   USB. A zero-length packet will return `Ok(0)`.
    /// * [`BufferOverflow`](crate::UsbError::BufferOverflow) - The received packet is too long to
    ///   fit in `data`. This is generally an error in the class implementation.
    fn read(&mut self, data: &mut [u8]) -> Result<usize> {
        if !self.is_initialized() {
            return Err(UsbError::InvalidEndpoint);
        }

        if self.address.direction() == UsbDirection::In {
            return Err(UsbError::InvalidEndpoint);
        }

        interrupt::free(|cs| self.buffer.unwrap().borrow(cs).borrow_mut().read_packet(data))
    }
}

/// Handle for IN endpoints.
impl EndpointIn for SynopsysEndpoint {
    /// Writes a single packet of data to the specified endpoint. The buffer must not be longer than
    /// the `max_packet_size` specified when allocating the endpoint.
    ///
    /// # Errors
    ///
    /// Note: USB bus implementation errors are directly passed through, so be prepared to handle
    /// other errors as well.
    ///
    /// * [`WouldBlock`](crate::UsbError::WouldBlock) - The transmission buffer of the USB
    ///   peripheral is full and the packet cannot be sent now. A peripheral may or may not support
    ///   concurrent transmission of packets.
    /// * [`BufferOverflow`](crate::UsbError::BufferOverflow) - The data is longer than the
    ///   `max_packet_size` specified when allocating the endpoint. This is generally an error in
    ///   the class implementation.
    fn write(&mut self, data: &[u8]) -> Result<()> {
        let ep = endpoint_in::instance(self.address.number());
        if !self.is_initialized() {
            return Err(UsbError::InvalidEndpoint);
        }

        if self.address.direction() == UsbDirection::Out {
            return Err(UsbError::InvalidEndpoint);
        }

        if self.address.number() != 0 && read_reg!(endpoint_in, ep, DIEPCTL, EPENA) != 0 {
            return Err(UsbError::WouldBlock);
        }

        if data.len() > self.max_packet_size() as usize {
            return Err(UsbError::BufferOverflow);
        }

        if !data.is_empty() {
            // Check for FIFO free space
            let size_words = (data.len() + 3) / 4;
            if size_words > read_reg!(endpoint_in, ep, DTXFSTS, INEPTFSAV) as usize {
                return Err(UsbError::WouldBlock);
            }
        }

        #[cfg(feature = "fs")]
        write_reg!(endpoint_in, ep, DIEPTSIZ, PKTCNT: 1, XFRSIZ: data.len() as u32);
        #[cfg(feature = "hs")]
        write_reg!(endpoint_in, ep, DIEPTSIZ, MCNT: 1, PKTCNT: 1, XFRSIZ: data.len() as u32);

        modify_reg!(endpoint_in, ep, DIEPCTL, CNAK: 1, EPENA: 1);

        fifo_write(self.address.number(), data);

        Ok(())
    }
}
