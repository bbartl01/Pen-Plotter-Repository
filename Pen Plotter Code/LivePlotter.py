'''!
    @file       LivePlotter.py
    
    @brief      File that creates a live animation of shape being drawn.  
    
    @details    This file receives data over UART to create a live animation of the Pen Plotter drawing the shape 
                that it is currently drawing.  

                
    @author     Baxter Bartlett
    @author     Jake Lesher
    @date       6/5/2022
    
'''

import serial, gc, math
from matplotlib import pyplot as plt
import numpy as np
import IPython

def Plot():
    '''! @brief A function that creates the live animation of the Pen Plotter drawing.  
         @details This function receives data from UART and uses it to update a plot, thus creating a real-time anmation 
                  of the pen plotter drawing the shape.  
    '''
    
    ser = serial.Serial('COM8',115200)
    ser.flush()
    while True:
        try:
            if ser.inWaiting()>0:
                # read a line of data
                data = ser.readline()
                # decode the line to get x location, y location, and angle of arm
                x_point, y_point, th_point = map(float, data.decode().strip('\r\n').split(','))
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
        

if __name__ == '__main__':
    
    ## @brief List of x-coordinates  
    #  @details List that is continually updated with x-coordinates.  Storing all x-coordinates is necessary such that 
    #           the sketch is displayed.  
    #
    x = []
    
    ## @brief List of y-coordinates  
    #  @details List that is continually updated with y-coordinates.  Storing all y-coordinates is necessary such that 
    #           the sketch is displayed.  
    #
    y = []

    ## @brief LArm length 
    #  @details Contains the length of the arm.  This is used to create the animation of the robot arm.  
    #
    arm = 290
    
    ## @brief Minimum x-distance for the plot.  
    #  @details Used to set the lower x-limit for the plot.  
    #
    xmin = -100
    
    ## @brief Maximum x-distance for the plot.  
    #  @details Used to set the upper x-limit for the plot.  
    #
    xmax = 200
    
    ## @brief Minimum y-distance for the plot.  
    #  @details Used to set the lower y-limit for the plot.  
    #
    ymin = 0
    
    ## @brief Maximum y-distance for the plot.  
    #  @details Used to set the upper y-limit for the plot.  
    #
    ymax = 300
    gc.collect()
    
    fig, ax = plt.subplots()
    ax.plot(1,1)
    ax.set_xlim(xmin,xmax)
    ax.set_ylim(ymin,ymax)
    ax.set_aspect(1)
    
    Plot()

