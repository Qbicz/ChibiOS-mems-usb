#!/usr/bin/python

import usb.core
import usb.util
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

class UsbLivePlot:

    def __init__(self):
    
        # find our device - Vendor ST, Product STM32F4
        self.usbDev = usb.core.find(idVendor=0x0483, idProduct=0xF00D)
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
        self.datasize = 16
        self.timear = []
        self.xar = []
        self.yar = []
        self.zar = []
        

    def xyzFromUsb(self, usbData):
        xbytes = usbData[0              :self.datasize]
        ybytes = usbData[self.datasize  :2*self.datasize]
        zbytes = usbData[2*self.datasize:3*self.datasize]
        #tbytes = usbData[3*self.datasize:(3+2)*self.datasize]
        
        #x = int.from_bytes(xbytes, byteorder='little', signed='false')
        #y = int.from_bytes(ybytes, byteorder='little', signed='false')
        #z = int.from_bytes(zbytes, byteorder='little', signed='false')
        # convert from (-128,+128) to g values; approx 54 lsb = 1g
        #x = x*0.0185
        #y = y*0.0185
        #z = z*0.0185
        
        return xbytes,ybytes,zbytes
        
    def usbReadToFile(self, filename):
        pass
        # TODO: save to file bigger chunks and from it load to plot

        
    def animate(self, i):
        
        # Read USB       
        try:
            usbData = self.usbDev.read(self.epIn.bEndpointAddress, 3*self.datasize, 100) # 100ms timeout
        except usb.core.USBError as e:
            print('Data not read:', e)
            return
            
        xbuf,ybuf,zbuf = self.xyzFromUsb(usbData)
        
        # TODO: timestamps should be created by MCU
        t = time.time() - self.startTime
        
        timegen = []
        for i in range(self.datasize):
            timegen.append(t + i*0.025) # seconds
        
        
        # count time in loop
        
        # Open log
        #file = open(self.filename, 'a')
        
        # Arrays to integers, time is uint16
        #for i in range(len(xbuf)):
            #x[i] = int.from_bytes(xbuf[i], byteorder='little', signed='false')
            #y[i] = int.from_bytes(ybuf[i], byteorder='little', signed='false')
            #z[i] = int.from_bytes(zbuf[i], byteorder='little', signed='false')
            #t[i] = int.from_bytes(tbuf[2*i:2*i+1], byteorder='little', signed='false')
            
            # -128 everywhere
            #file.write('%.3f %d %d %d\n' % (t[i],x[i],y[i],z[i]))
          
        #file.close()
        
        self.timear.append(timegen)
        self.xar.append(xbuf)
        self.yar.append(ybuf)
        self.zar.append(zbuf)
               
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax1.plot(self.timear, self.xar)
        self.ax2.plot(self.timear, self.yar)
        self.ax3.plot(self.timear, self.zar)
        
        self.ax1.set_title('Acceleration from STM32F4Discovery')
        self.ax1.set_ylabel('x-axis [g]')
        self.ax2.set_ylabel('y-axis [g]')
        self.ax3.set_ylabel('z-axis [g]')
        self.ax3.set_xlabel('Time [s]')
        
        
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
    usbLive.filename = 'acceleration.log'
    
    # Create a self-updating plot
    ani = animation.FuncAnimation(usbLive.fig, usbLive.animate, interval = 1)
    plt.show()
    

if __name__ == '__main__':
    main()
