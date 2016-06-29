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
        self.ax1 = self.fig.add_subplot(2,1,1)
        self.ax2 = self.fig.add_subplot(2,1,2)
        self.startTime = time.time()
        
        # create buffers
        self.timear = []
        self.xar = []
        self.yar = []
        

    def xyFromUsb(self, usbData):
        xbytes = usbData[0:4]
        ybytes = usbData[4:8]

        x = int.from_bytes(xbytes, byteorder='little', signed='false')
        y = int.from_bytes(ybytes, byteorder='little', signed='false')
        
        # TODO: Scale to G (full res is 2G)
        # 1G for ~53
        
        return x,y
        

    def animate(self, i):
        
        # USB read
        usbData = self.usbDev.read(0x81, 8)
        x,y = self.xyFromUsb(usbData)
        t = time.time() - self.startTime
        
        self.timear.append(float(t))
        self.xar.append(float(x))
        self.yar.append(float(y))
               
        self.ax1.clear()
        self.ax2.clear()
        self.ax1.plot(self.timear, self.xar)
        self.ax2.plot(self.timear, self.yar)
        
        self.ax1.set_title('Acceleration from STM32F4Discovery')
        self.ax2.set_xlabel('Time [s]')
        self.ax1.set_ylabel('x-axis acceleration')
        self.ax2.set_ylabel('y-axis acceleration')
        
        
    def findEndpoint(self):
        cfg = self.usbDev.get_active_configuration()
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
        
    def logToFile(self, filename):
        pass
        # TODO: use usb logger or simply put data to file
    
def main():
    
    usbLive = UsbLivePlot()

    # TODO: add reading z axis and convert to degrees


    # Create a self-updating plot
    ani = animation.FuncAnimation(usbLive.fig, usbLive.animate, interval = 50)
    plt.title('STM32F4 Discovery accelerometers')
    plt.show()
    

if __name__ == '__main__':
    main()
