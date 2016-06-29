#!/usr/bin/python

import usb.core
import usb.util
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

class UsbLivePlot:

    def __init__(self):
    
        # find our device - Vendor ST, Product STM32F4
        self.usbDev = usb.core.find(idVendor=0x0483, idProduct=0xBABE)
        if self.usbDev is None:
            raise ValueError('Device not found')
            
        # With no arguments, the first configuration will be the active one
        self.usbDev.set_configuration()
            
        # create figure with two subplots
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(3,1,1)
        self.ax2 = self.fig.add_subplot(3,1,2)
        self.ax3 = self.fig.add_subplot(3,1,3)
        self.startTime = time.time()
        
        # TODO: add angles subplots on the right side of the window
        
        # create buffers
        self.timear = []
        self.xar = []
        self.yar = []
        self.zar = []
        

    def xyFromUsb(self, usbData):
        xbytes = usbData[0:4]
        ybytes = usbData[4:8]
        zbytes = usbData[9:12]
        # TODO: change magic numbers to well-named sizes

        x = int.from_bytes(xbytes, byteorder='little', signed='false')
        y = int.from_bytes(ybytes, byteorder='little', signed='false')
        z = int.from_bytes(zbytes, byteorder='little', signed='false')
        return x,y,z
        

    def animate(self, i):
        
        # USB read
        usbData = self.usbDev.read(0x81, 12)
        x,y,z = self.xyFromUsb(usbData)
        # TODO: timestamps should be created by MCU
        t = time.time() - self.startTime
        
        self.timear.append(float(t))
        self.xar.append(float(x))
        self.yar.append(float(y))
        self.zar.append(float(z))
               
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax1.plot(self.timear, self.xar)
        self.ax2.plot(self.timear, self.yar)
        self.ax3.plot(self.timear, self.zar)
        
        self.ax1.set_title('Acceleration from STM32F4Discovery')
        self.ax2.set_xlabel('Time [s]')
        self.ax1.set_ylabel('x-axis acceleration')
        self.ax2.set_ylabel('y-axis acceleration')
        self.ax3.set_ylabel('z-axis acceleration')
        
        
    def findEndpoint(self):
        cfg = self.usbDev.get_active_configuration()
        intf = cfg[(0,0)]

        ep = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                usb.util.ENDPOINT_IN)

        assert ep is not None
        print(ep)
        
    
def main():
    
    usbLive = UsbLivePlot()

    # Create a self-updating plot
    ani = animation.FuncAnimation(usbLive.fig, usbLive.animate, interval = 50) # TODO: use USB data ready interrupt
    plt.title('STM32F4 Discovery accelerometers')
    plt.show()
    

if __name__ == '__main__':
    main()
