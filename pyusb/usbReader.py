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
        
        # find and assign IN endpoint
        self.epIn = self.findEndpoint(usb.util.ENDPOINT_IN)
            
        # create figure with two subplots
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(3,1,1)
        self.ax2 = self.fig.add_subplot(3,1,2)
        self.ax3 = self.fig.add_subplot(3,1,3)
        self.startTime = time.time()
        
        # TODO: add angles subplots on the right side of the window
        
        # create buffers
        self.datasize = 1
        self.timear = []
        self.xar = []
        self.yar = []
        self.zar = []
        

    def xyzFromUsb(self, usbData):
        xbytes = usbData[0              :self.datasize]
        ybytes = usbData[self.datasize  :2*self.datasize]
        zbytes = usbData[2*self.datasize:3*self.datasize]

        x = int.from_bytes(xbytes, byteorder='little', signed='false')
        y = int.from_bytes(ybytes, byteorder='little', signed='false')
        z = int.from_bytes(zbytes, byteorder='little', signed='false')
        # convert from (-128,+128) to g values; approx 54 lsb = 1g
        x = x/54
        y = y/54
        z = z/54
        
        return x,y,z
        
    def usbReadToFile(self, filename):
        pass
        # TODO: save to file bigger chunks and from it load to plot

    def animate(self, i):
        
        # USB read
        usbData = self.usbDev.read(self.epIn.bEndpointAddress, 3*self.datasize)
        x,y,z = self.xyzFromUsb(usbData)
        # TODO: timestamps should be created by MCU
        t = time.time() - self.startTime
        
        self.timear.append(float(t))
        self.xar.append(x)
        self.yar.append(y)
        self.zar.append(z)
               
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
        
        
    def findEndpoint(self, direction):
        cfg = self.usbDev.get_active_configuration()
        intf = cfg[(0,0)]

        ep = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                direction)

        assert ep is not None
        print(ep)
        return ep
        
    
def main():
    
    usbLive = UsbLivePlot()

    # Create a self-updating plot
    ani = animation.FuncAnimation(usbLive.fig, usbLive.animate, interval = 5) # TODO: use USB data ready interrupt
    plt.title('STM32F4 Discovery accelerometers')
    plt.show()
    

if __name__ == '__main__':
    main()
