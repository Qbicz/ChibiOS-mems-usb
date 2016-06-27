#!/usr/bin/python

import usb.core
import usb.util
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# create figure with two subplots
fig = plt.figure()
ax1 = fig.add_subplot(2,1,1)
ax2 = fig.add_subplot(2,1,2)

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

xbytes = usbData[0:4]
ybytes = usbData[4:8]

x = int.from_bytes(xbytes, byteorder='little', signed='false')
y = int.from_bytes(ybytes, byteorder='little', signed='false')

# TODO: add reading z axis and convert to degrees

print(x)
print(y)

