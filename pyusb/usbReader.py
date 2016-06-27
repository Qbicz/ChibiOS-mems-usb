#!/usr/bin/python

import usb.core
import usb.util
import sys

# find our device - Vendor ST, Product STM32F4
dev = usb.core.find(idVendor=0x0483, idProduct=0xBABE)

if dev is None:
    raise ValueError('Device not found')

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.set_configuration()

cfg = dev.get_active_configuration()
intf = cfg[(0,0)]

ep = usb.util.find_descriptor(
    intf,
    #match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_IN)

assert ep is not None
print(ep)

# not using ep yet
size = 8
#timeout = 
usbData = dev.read(0x81, size)

#usbData = ep.read()
print(usbData)

x = usbData[0:4]
y = usbData[4:8]

print(x)
print(y)

