use crate::endpoint_memory::EndpointMemoryAllocator;
use crate::endpoint_trait::SynopsysEndpoint;
use usb_device::allocator::EndpointConfig;
use usb_device::bus::EndpointAllocator;
use usb_device::bus::UsbBus;
use usb_device::endpoint::{Endpoint, EndpointAddress};
use usb_device::UsbDirection;
use usb_device::{Result, UsbError};

pub struct SynopsysEndpointAllocator {
    pub endpoints_in: [SynopsysEndpoint; 4],
    pub endpoints_out: [SynopsysEndpoint; 4],
    pub memory_allocator: EndpointMemoryAllocator,
}

impl SynopsysEndpointAllocator {
    pub fn new(ep_memory: &'static mut [u32]) -> SynopsysEndpointAllocator {
        SynopsysEndpointAllocator {
            endpoints_in: [
                SynopsysEndpoint::new(EndpointAddress::from_parts(0, UsbDirection::In)),
                SynopsysEndpoint::new(EndpointAddress::from_parts(1, UsbDirection::In)),
                SynopsysEndpoint::new(EndpointAddress::from_parts(2, UsbDirection::In)),
                SynopsysEndpoint::new(EndpointAddress::from_parts(3, UsbDirection::In)),
            ],
            endpoints_out: [
                SynopsysEndpoint::new(EndpointAddress::from_parts(0, UsbDirection::Out)),
                SynopsysEndpoint::new(EndpointAddress::from_parts(1, UsbDirection::Out)),
                SynopsysEndpoint::new(EndpointAddress::from_parts(2, UsbDirection::Out)),
                SynopsysEndpoint::new(EndpointAddress::from_parts(3, UsbDirection::Out)),
            ],
            memory_allocator: EndpointMemoryAllocator::new(ep_memory),
        }
    }

    pub fn find_free_endpoint(
        &mut self,
        direction: UsbDirection,
        ep_addr: Option<EndpointAddress>,
    ) -> Result<SynopsysEndpoint> {
        let endpoints: [SynopsysEndpoint; 4] = match direction {
            UsbDirection::In => self.endpoints_in,
            UsbDirection::Out => self.endpoints_out,
        };

        if let Some(address) = ep_addr {
            for i in 0..4 {
                if endpoints[i].address() == address {
                    if !endpoints[i].is_initialized() {
                        return Ok(endpoints[i]);
                    } else {
                        return Err(UsbError::InvalidEndpoint);
                    }
                }
            }

            Err(UsbError::InvalidEndpoint)
        } else {
            for i in 1..4 {
                if !endpoints[i].is_initialized() {
                    return Ok(endpoints[i]);
                }
            }
            Err(UsbError::EndpointOverflow)
        }
    }
}

impl<B> EndpointAllocator<B> for SynopsysEndpointAllocator
where
    B: UsbBus<EndpointIn = SynopsysEndpoint, EndpointOut = SynopsysEndpoint>,
{
    /// Allocates an OUT endpoint with the provided configuration
    fn alloc_out(&mut self, config: &EndpointConfig) -> Result<B::EndpointOut> {
        let ep = self.find_free_endpoint(
            UsbDirection::Out,
            config
                .number
                .map(|num| EndpointAddress::from_parts(num, UsbDirection::In)),
        )?;

        let buffer = self
            .memory_allocator
            .allocate_rx_buffer(config.max_packet_size as usize)?;

        ep.initialize(UsbDirection::Out, config, Some(buffer));

        Ok(ep)
    }

    /// Allocates an IN endpoint with the provided configuration
    fn alloc_in(&mut self, config: &EndpointConfig) -> Result<B::EndpointIn> {
        let ep = self.find_free_endpoint(
            UsbDirection::In,
            config
                .number
                .map(|num| EndpointAddress::from_parts(num, UsbDirection::In)),
        )?;
        ep.initialize(UsbDirection::In, config, None);

        Ok(ep)
    }
}
