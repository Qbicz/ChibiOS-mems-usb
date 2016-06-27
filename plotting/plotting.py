import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

fig = plt.figure()
ax1 = fig.add_subplot(2,1,1)
ax2 = fig.add_subplot(2,1,2)

def animate(i):
    pullData = open('plotData.txt', 'r').read()
    dataArray = pullData.split('\n')
    timear = []
    xar = []
    yar = []
    for line in dataArray:
        if len(line) > 2:
            t,x,y = line.split(',')
            
            timear.append(float(t))
            xar.append(float(x))
            yar.append(float(y))
    
    ax1.clear()       
    ax1.plot(timear,xar)
    ax2.clear()
    ax2.plot(timear,yar)
    
ani = animation.FuncAnimation(fig, animate, interval = 1000)

plt.title('STM32F4 Discovery accelerometers')
plt.show()

