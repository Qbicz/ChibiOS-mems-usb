#!/usr/bin/python

import usb.core
import usb.util

# find our device - Vendor ST, Product STM32F4
# dev = usb.core.find(idVendor=0x0483, idProduct=0x5740)



printers = usb.core.find(find_all=True, bDeviceClass=7)

# Python 2, Python 3, to be or not to be
import sys
sys.stdout.write('There are ' + len(printers) + ' in the system\n.')



# was it found?
#if usb.core.find(bDeviceClass=2) is None:
#    raise ValueError('No CDC found')

if dev is None:
    raise ValueError('Device not found')

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.set_configuration()

# get an endpoint instance
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]

ep = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_OUT)

assert ep is not None

# write the data
ep.write('test')
