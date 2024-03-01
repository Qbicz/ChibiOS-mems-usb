# ChibiOS-mems-usb project

In this repository you can find code for STM32F4 which communicates with onboard MEMS accelerometer LIS302DL (on Discovery board) and sends values over USB bulk endpoint.

The data is plotted on PC in real-time, as the data appears on the USB bus.

![alt tag](doc/livePlotExample.png)


# Quick start for Windows

1. Clone project
2. Grab your STM32F4-Discovery and flash it with `bin/STM32/ch.elf`
3. Download https://sourceforge.net/projects/libusb-win32/ and use its `inf-wizard.exe` to install a driver for the MCU
4. Run 'python/usbReader.exe' to watch the live plots! [To be prepared]


# ChibiOS
STM32 runs elegant and simple RTOS, ChibiOS/RT (https://www.chibios.org/dokuwiki/). ChibiOS project also provides HAL and drivers for all major hardware interfaces.


# Live Plot
In `python` directory a class for plotting the data gathered from USB is supplied. It depends on python3, pyUSB (https://github.com/walac/pyusb) and matplotlib (http://matplotlib.org).

```python
cd python
pip3 install pyusb
pip3 install matplotlib
python3 usbReader.py
```

# Debugging
USB bus activity was sniffed and debugged using USBlyzer 2.0. (http://www.usblyzer.com/)


# Data flow

MEMS accelerometer --(SPI)--> STM32F4 --(USB FS)--> PC --> plot
