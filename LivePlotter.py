import time, array, serial, gc, math
from matplotlib import pyplot as plt
# from matplotlib.animation import FuncAnimation
import numpy as np
import IPython

def Plot():
#    fig, ax = plt.subplots()
#    ax.set_aspect('equal',adjustable='box')
    ser = serial.Serial('COM8',115200)
    ser.flush()
    while True:
        try:
            if ser.inWaiting()>0:
                # read a line of data
                data = ser.readline()
                #print('The data sent is',data,len(data))
                # decode the line to get x location, y location, and angle of arm
                x_point, y_point, th_point = map(float, data.decode().strip('\r\n').split(','))
                #print(x_point,y_point,th_point)
                # Add these points to a list
                x.append(x_point)
                y.append(y_point)
                x_arm = np.linspace(0,arm*math.cos(th_point),num = 5000)
                y_arm = np.linspace(0,arm*math.sin(th_point),num = 5000)
                # clear axes
                ax.cla()
                # plot the sketch
                ax.plot(x,y,'ro',color='k',markersize = 2)
                # plot the robot's arm
                ax.plot(x_arm,y_arm,color='g',linewidth=3)
                # plot the location of the pen on the robot's arm
                ax.plot(x_point,y_point,marker='.',markersize=20,color='k')
                ax.set_xlim(xmin,xmax)
                ax.set_ylim(ymin,ymax)
                IPython.display.display(fig)
                print('Updating')
        except KeyboardInterrupt:
            break


# def animate():
#     first = True
#     with serial.Serial('COM8', 115200, timeout=5) as ser: # separate 
#         if ser.any():
#             if first == True:
#                 ser.flush()
#                 first = False
#             data = ser.readline()
#             x_point, y_point, th_point = map(float, data.decode().strip('\r\n').split(','))
#             x.append(x_point)
#             y.append(y_point)
#             th.append(th_point)

# def LivePlot():
#     ani = FuncAnimation(plt.gcf(), animate, interval = 1000) # interval based off trial and error
        

if __name__ == '__main__':
    x = []
    y = []

    # length of arm
    arm = 290
    xmin = -100
    xmax = 200
    ymin = 0
    ymax = 300
    gc.collect()
    
    fig, ax = plt.subplots()
    ax.plot(1,1)
    ax.set_xlim(xmin,xmax)
    ax.set_ylim(ymin,ymax)
    ax.set_aspect(1)
    
    Plot()

