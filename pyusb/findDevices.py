#!/usr/bin/python
import sys
import usb.core

# find USB devices
dev = usb.core.find(find_all=True)

# formatted device list
for device in dev:
    print(device)
  