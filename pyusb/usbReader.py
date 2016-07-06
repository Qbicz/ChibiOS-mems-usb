#!/usr/bin/python

import usb.core
import usb.util
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

class UsbLivePlot:
    
    """
    UsbLivePlot class provides a way to receive sensor readings in form of USB packets and create a self-refreshing plot using matplotlib FuncAnimation.

    Filip Kubicz 2016
    """

    def __init__(self):
    
    
        # find our device - Vendor ST, Product STM32F4
        self.usbDev = usb.core.find(idVendor=0x0483, idProduct=0xF00D)
        if self.usbDev is None:
            raise ValueError('Device not found')
            
        # With no arguments, the first configuration will be the active one
        self.usbDev.set_configuration()
        
        # find and assign IN endpoint
        self.epIn = self.findEndpoint(usb.util.ENDPOINT_IN)
            
        # create figure with x,y,z subplots
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(3,1,1)
        self.ax2 = self.fig.add_subplot(3,1,2)
        self.ax3 = self.fig.add_subplot(3,1,3)
        self.startTime = time.time()
        
        # TODO: add angles subplots on the right side of the window
        
        # create buffers
        self.datasize = 12
        self.timear = []
        self.xar = []
        self.yar = []
        self.zar = []
        

    def xyzFromUsb(self, usbData):
        xbytes = usbData[0              :self.datasize]
        ybytes = usbData[self.datasize  :2*self.datasize]
        zbytes = usbData[2*self.datasize:3*self.datasize]
        tbytes = usbData[3*self.datasize:(3+2)*self.datasize]
        
        x = []
        y = []
        z = []
        t = []
        # Byte arrays to integers, time is uint16
        for i in range(len(xbytes)):
            x.append(self.accel_byte2g(xbytes[i]))
            y.append(self.accel_byte2g(ybytes[i]))
            z.append(self.accel_byte2g(zbytes[i]))
            
            t_tmp = int.from_bytes(tbytes[2*i:2*i+2], byteorder='little', signed=False)
            t.append(t_tmp/1000) # seconds
        
        return x,y,z,t # little endian transmission
        
        
    def accel_byte2g(self, accel_byte):
        """
        Convert unsigned char data to acceleration in [g]
        """
        accel_g = accel_byte - 256 if accel_byte > 127 else accel_byte
        accel_g = float(accel_g) * 0.0185
        return accel_g
        
        
    def animate(self, i):
        
        # Read USB    
        timeout = 50
        try:
            usbData = self.usbDev.read(self.epIn.bEndpointAddress, 5*self.datasize, timeout)
        except usb.core.USBError as e:
            print('Data not read:', e)
            return
            
        xbuf,ybuf,zbuf,tMcu = self.xyzFromUsb(usbData)
        
        #t = time.time() - self.startTime
        
        #timegen = []
        #for i in range(self.datasize):
        #    timegen.append(t + i*0.025) # seconds
        
        
        # use extend() instead of append() to have 1-D list where plot lines make sense
        # note: data obtained from
        self.timear.extend(tMcu)
        self.xar.extend(xbuf)
        self.yar.extend(ybuf)
        self.zar.extend(zbuf)
               
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax1.plot(self.timear, self.xar, marker='.', linestyle='None')
        self.ax2.plot(self.timear, self.yar, marker='.', linestyle='None', color='orange')
        self.ax3.plot(self.timear, self.zar, marker='.', linestyle='None', color='green')
        
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
    ani = animation.FuncAnimation(usbLive.fig, usbLive.animate, interval = 30)
    plt.show()
    

if __name__ == '__main__':
    main()
