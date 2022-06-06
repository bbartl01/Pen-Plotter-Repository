'''!
    @file       main.py
    
    @brief      File that controls motors for term project.
    
    @details    This file performs all operations necessary to control the Pen Plotter such that it draws a desired shape.  
                It parses the HPGL code, performs selective interpolation, uses a Newton-Raphson method to compute motor angles, 
                and sends motor commands to the motors.  

                
    @author     Baxter Bartlett
    @author     Jake Lesher
    @date       6/5/2022
    
'''

import pyb, stepperdriver, actuator, cotask, micropython, gc, math, task_share
from ulab import numpy as np
from pyb import SPI, Pin, UART, repl_uart

def linspace(a,b,n):
    '''! @brief Special interpolation function that creates a list of interpolated points excluding the last point.  
         @details This function interpolates a specified number of points between two points.  It then places all those points 
                  in a list, but excludes the last point.  
         @param a The first point to interpolate between.
         @param b The second point to interpolate between.  
         @param n Number of points plus one for which the interpolated list should contain.  
         @return A list of interpolated points excluding b.  
    '''
    diff = (b-a)/(n-1)
    return [diff*i+a for i in range(n-1)]

# First Function
def g(x, theta):
    '''! @brief A function computng the difference between a desired location and current location.  
         @details This function is used in the Netwon-Raphson method to determine when to stop iterating.  
         @param x A 2x1 matrix containing the desired location for a round of Newton-Raphson.  
         @param theta A 2x1 matrix containing the current position for an iteration of Newton-Raphson.  
         @return A 2x1 matrix containing the difference between the desired and current position.  
    '''
    f = np.array(([((4/math.pi)*theta[1][0])*math.cos(theta[0][0]/3)],
                  [((4/math.pi)*theta[1][0])*math.sin(theta[0][0]/3)]))
    return x - f

# Second Function
def dg_dtheta(theta):
    '''! @brief A function computng the Jacobian of our Pen Plotter.  
         @details This function is used in compute the Jacobian of our Pen Plotter during Newton-Raphson.  
         @param theta A 2x1 matrix containing the current location for an iteration of Newton-Raphson.  
         @return A 2x2 matrix containing the Jacobian of the system.  
    '''
    return np.array(([(1/3)*((4/(math.pi))*theta[1][0])*math.sin(theta[0][0]/3), (-4/math.pi)*math.cos(theta[0][0]/3)],
                     [(-1/3)*((4/(math.pi))*theta[1][0])*math.cos(theta[0][0]/3), (-4/math.pi)*math.sin(theta[0][0]/3)]))

# Third Function
def NewtonRaphson(fcn, jacobian, guess, thresh):
    '''! @brief Determines the motor angles required to achieve a desired x and y position.  
         @details This function uses a Newton-Raphson method to determine the motor angles required to achieve a desired x and y position.   
         @param fcn Placeholder for the difference function (g) to be passed into the function.  fcn must be a lambda function that passes the 
                     the desired location, effectively making g a function of the motor angles solely.  
         @param jacobian Passes in the jacobian function.  
         @param guess A 2x1 matrix containing an initial motor angle guess to start iteration.  
         @param thresh A number for which g should be less than to stop iteration.  
         @return The motor angles required to achieve a given position.   
    '''
    theta_n = guess
    end_NR = 0
    while end_NR==0:
        g = fcn(theta_n)
        # Condition in which g is less than the threshhold, triggering the algorithm to stop
        if abs(g[0][0]) <= thresh and abs(g[1][0]) <= thresh:
            end_NR = 1
        # Newton-Raphson Algorithm
        else:
            theta_n = theta_n - np.dot(np.linalg.inv(jacobian(theta_n)),g)
        
    return theta_n


def startup():
    '''! @brief A function that performs all calculations before the motors start drawing.  
         @details This function is called at the start of the drawing process.  It parses the HPGL code, performs selective interpolation, 
                  and executes the Newton-Raphon method on every location to obtain the required motor angles.  To store the immense 
                  amount of data points, two text files are used: one to store interpolated points and one to store points obtained via 
                  Newton-Raphson.  
         @return The number of data points.  
    '''
    
    print('Parsing HPGL Code')
    
    # Parsing file data into a readable list of tuples.
    #filename = 'Star&Spiral.hpgl'
    #filename = 'Lab3Square.hpgl'
    #filename = 'SLO.hpgl'
    filename = 'JB.hpgl'
    #filename = 'Boxes.hpgl'
    #filename = 'block_names.hpgl'
    
    with open(filename, 'r') as f:
        command_list = f.readline()
    command_list = command_list.split(';')
    STEPS = []
    
    for idx, command in enumerate(command_list):
        gc.collect()
        if "PU" in command:
            if ',' in command:
                coords = command.strip('PU').split(',')
                xbool = 1
                for coord in coords:
                    if xbool == 1:
                        x=int(coord)
                        xbool = 0
                    else:
                        y=int(coord)
                        tup = ('PU', x, y)
                        STEPS.append(tup)
                        xbool = 1
            
        elif "PD" in command:
            if ',' in command:
                coords = command.strip('PD').split(',')
                x_bool = 1
                for coord in coords:
                    if x_bool == 1:
                        x=int(coord)
                        x_bool = 0
                    else:
                        y=int(coord)
                        tup = ('PD', x, y)
                        STEPS.append(tup)
                        x_bool = 1
    
    print('HPGL Code Sucessfully Parsed')
    
    print('')
    
    print('Performing Necessary Interpolations')
    
    command_list = ""
    gc.collect()
    interp_file = "Interpolated.txt"
    print('')
    N = 3
    gc.collect()
    max_dist = 10
    elements = 0
    
    interp = open(interp_file,'w')
    

    for idx, coords in enumerate(STEPS):
        # We have two conditions: 'PU' and 'PD'
        if coords[0] == 'PU':
            # Since we compute linspace between two elements, we need to check if the nextstep
            # element exists, as it might not if coords[0]='PU'
            if idx+1 < len(STEPS):
                # nextstep is the next position after the current position
                nextstep = STEPS[idx+1]
                # next_dist computes the distance between the current position and the next position
                next_dist = ((nextstep[1]-coords[1])**(2)+(nextstep[2]-coords[2])**(2))**(1/2)
                # If the distance between the current point and the next point exceeds the
                # maximum allowable distance, we need to interpolate
                if next_dist > max_dist:
                    xA = linspace(coords[1],nextstep[1],N)
                    yA = linspace(coords[2],nextstep[2],N)
                    first = True
                    for i,j in zip(xA,yA):
                        # After interpolating between a 'PU' and a 'PD', only the first
                        # item should be a 'PU' - everything else should be a 'PD'
                        if first == True:
                            first = False
                            interp.write(f"PU, {i}, {j}\r\n")
                            elements += 1
                        else:
                            interp.write(f"PD, {i}, {j}\r\n")
                            elements += 1
                # If the distance is less than the maximum allowable distance, we do not
                # need to interpolate - we just need to add the current position to the list
                else:
                    interp.write(f"PU, {coords[1]}, {coords[2]}\r\n")
                    elements += 1
            # The only point for which idx+1 > len(STEPS) is the last point.  We simply
            # add this point to the list
            else:
                interp.write(f"PU, {coords[1]}, {coords[2]}\r\n")
                elements += 1
        # Now we deal with 'PD'
        elif coords[0] == 'PD':
            # Since STEPS will never end with a 'PD', we know that a nextstep exists
            nextstep = STEPS[idx+1]
            next_dist = ((nextstep[1]-coords[1])**(2)+(nextstep[2]-coords[2])**(2))**(1/2)
            # We only need to interpolate between two 'PD' points as we do not care if the
            # path is jagged when going from a 'PD' point to a 'PU' point
            if nextstep[0] == 'PD':
                if next_dist > max_dist:
                    xA = linspace(coords[1],nextstep[1],N)
                    yA = linspace(coords[2],nextstep[2],N)
                    for i,j in zip(xA,yA):
                        interp.write(f"PD, {i}, {j}\r\n")
                        elements += 1
                # add current position to list if distance does not exceed maximum distance
                else:
                    interp.write(f"PD, {coords[1]}, {coords[2]}\r\n")
                    elements += 1
            # Do not interpolate if next position is 'PU'
            else:
                interp.write(f"PD, {coords[1]}, {coords[2]}\r\n")
                elements += 1
        else:
            print('Did not get rid of SP or IN')


    interp.close()
    print('Finished Interpolating')
    
    print('')
    
    print(elements, 'elements total')
    
    print('')
    
    print('Peforming Newton-Raphson Method')
    
    gc.collect()
    xSCALE = 1/40 # Scales the units of the hpgl file.
    xOFFSET = -70
    ySCALE = 1/40
    yOFFSET = -500

    i = 0
    
    send_file = "Send.txt"
    
    interp = open(interp_file,'r')
    send = open(send_file,'w')

    # Initial guess of motor angles to start the algorithm
    guess = np.array(([3*math.pi/2],
                      [110*2*math.pi/8]))
    while i<(elements):
        
        line = interp.readline()
        
        loc, x, y = line.split(',')
        x = float(x)
        y = float(y)
        
        # Apply offsets
        xS = xSCALE*x + xOFFSET
        yS = ySCALE*y + yOFFSET
        
        #print("Inside while loop")
        # Desired location on current iteration
        x_des = np.array(([xS],
                          [yS]))
#                 x_des = np.array(([xR[i]],
#                                  [yR[i]]))
        # For the first run of this algorithm, we want to use the initial guess specified
        # outside of this algorithm
        if i == 0:
            theta_guess = guess
        # For the remaining runs, we want to use the previous run's motor angles as our
        # initial guess, as this guess will be really close to the desired angles for the
        # current run (assuming that the shape is continuous)
        else:
            theta_guess = np.array(([th1],
                                    [th2]))
        #print("Theta_Guess calculated")
        theta = NewtonRaphson(lambda theta: g(x_des, theta),
                              dg_dtheta, theta_guess, 1e-3)
        print("NR angle computed")
        # Motor 1 Angle
        th1 = theta[0][0] % (2*math.pi)
        # Motor 2 Angle
        th2 = theta[1][0] 
        # Angle of arm
        th = th1/3
        
        send.write(f"{loc}, {th1}, {th2}, {xS}, {yS}, {th}\r\n")
        i+=1
    
    interp.close()
    send.close()
    print("Finished with NR")
    return elements

def task_main():
    
    '''! @brief A task that controls the motors to draw the desired shape.  
         @details This function reads from the text file containing the points obtained through the Newton-Raphson method.  
                  It sends these points to the motors if the motors have arrived at the previous position.  It also sends pen-up 
                  and pen down commands to the linear actuator and writes x location, y location, and arm angle over UART such that 
                  a live animation of the robot drawing can be made.  
    '''
    
    S0_INIT = micropython.const(0)
    S1_PLOT = micropython.const(1)
    S2_DONE = micropython.const(2)
    
    send_file = "Send.txt"
    
    uart2 = UART(2, 115200, 8, 0, 1)
    
    state = S0_INIT
    
    while True:
        
        if state == S0_INIT:
            step = 0
            pen_is_up = False
            pos_tolerance = 0
            update = True
            send = open(send_file,'r')
            state = S1_PLOT
            
        elif state == S1_PLOT:
            
            if step < elements.get() - 1:
                if update == True:
                    print('Send coordinate updated')
                    update = False
                    data = send.readline()
                    pen_CMD, th1, th2, xS, yS, th = data.split(',')
                    m1_CMD = float(th1)
                    m2_CMD = float(th2)
                    xS = float(xS)
                    yS = float(yS)
                    th = float(th)
                    uart2.write(f"{xS}, {yS}, {th}\r\n")
                
                if pen_CMD == 'PU' and pen_is_up == False:
                    pen.pen_up()
                    pen_is_up = True
                    motor1.set_target_pos(m1_CMD)
                    motor2.set_target_pos(m2_CMD)
                    if motor1.arrived(pos_tolerance) and motor2.arrived(pos_tolerance):
                        step+=1
                        update = True
                elif pen_CMD == 'PU' and pen_is_up == True:
                    motor1.set_target_pos(m1_CMD)
                    motor2.set_target_pos(m2_CMD)
                    if motor1.arrived(pos_tolerance) and motor2.arrived(pos_tolerance):
                        step+=1
                        update = True
                elif pen_CMD == 'PD' and pen_is_up == True:
                    pen.pen_down()
                    pen_is_up = False
                    motor1.set_target_pos(m1_CMD)
                    motor2.set_target_pos(m2_CMD)
                elif pen_CMD == 'PD' and pen_is_up == False:
                    motor1.set_target_pos(m1_CMD)
                    motor2.set_target_pos(m2_CMD)
                    if motor1.arrived(pos_tolerance) and motor2.arrived(pos_tolerance):
                        step+=1
                        update = True
                        
            elif step == elements.get() -1 :
                print("All done!")
                pen.pen_up()
                motor1.set_target_pos(3*4*math.pi/9)
                motor2.set_target_pos(105*math.pi/4)
                state = S2_DONE
                send.close()
                printed = False
                
        elif state == S2_DONE:
            if printed == False:
                printed = True
                print("Completed")
        
        else:
            raise ValueError("Invalid state.")
        
        yield state

if __name__ == '__main__':
    repl_uart(None)
    micropython.alloc_emergency_exception_buf(100)
    
    ## @brief Creates a clock pin
    #  @details The clock pin is used to create a timer.  
    #
    CLK_pin = Pin.cpu.B7
    
    ## @brief Creates an SPI clock pin
    #  @details The clock pin is used to control the rate that data is sent in SPI.   
    #
    SPI_CLK_pin = Pin.cpu.B13
    
    ## @brief Creates a MISO pin
    #  @details The pin is used as a MISO line in SPI 
    #
    MISO_pin = Pin.cpu.B14
    
    ## @brief Creates a MOSI pin
    #  @details The pin is used as a MOSI line in SPI 
    #
    MOSI_pin = Pin.cpu.B15
    
    ## @brief Creates a chip select pin for the first motor.
    #  @details The clock pin is used to indicate which motor to read/write from in SPI.  
    #
    CS1_pin = Pin.cpu.C3
    
    ## @brief Creates a chip select pin for the second motor.
    #  @details The clock pin is used to indicate which motor to read/write from in SPI.  
    #
    CS2_pin = Pin.cpu.C2
    
    ## @brief Creates an Enable pin to interface with the first TMC 2208.
    #  @details Each motor requires a TMC 2208 IC, which requires an Enable Pin.  
    #
    ENN1_pin = Pin.cpu.C0
    
    ## @brief Creates an Enable pin to interface with the second TMC 2208.
    #  @details Each motor requires a TMC 2208 IC, which requires an Enable Pin.  
    #
    ENN2_pin = Pin.cpu.B0
    
    # Creating CLK - Brown Wire
    
    ## @brief Creates timer.  
    #  @details The timer is used to control the rate of data transfer.  
    #
    tim = pyb.Timer(4, period=3, prescaler=0)
    
    ## @brief Configures the timer.    
    #  @details The timer is configured as a clock to facillitate data transfer.   
    #
    clk = tim.channel(2, pin=CLK_pin, mode=pyb.Timer.PWM, pulse_width=2)
    
    # Initializing chip_select_not pin. CS1=Orange and CS2=Yellow wires.
    
    ## @brief Configures the chip select pin for the first motor.  
    #  @details Configures the chip select pin as a Push-Pull.   
    #
    nCS1 = Pin(CS1_pin, mode=Pin.OUT_PP, value=1)
    
    ## @brief Configures the chip select pin for the second motor.  
    #  @details Configures the chip select pin as a Push-Pull.   
    #
    nCS2 = Pin(CS2_pin, mode=Pin.OUT_PP, value=1)
    
    # Initializing enable_not pin. ENN1=Gray and ENN2=White wires.
    
    ## @brief Configures the Enable pin for the first motor.  
    #  @details Configures the Enable pin as a Push-Pull.   
    #
    ENN1 = Pin(ENN1_pin, mode=Pin.OUT_PP, pull=Pin.PULL_NONE)
    
    ## @brief Configures the Enable pin for the second motor.  
    #  @details Configures the Enable pin as a Push-Pull.   
    #
    ENN2 = Pin(ENN2_pin, mode=Pin.OUT_PP, pull=Pin.PULL_NONE)
    
    # Creating SPI object on BUS2. SCK, MISO, MOSI = Green, Blue, Purple wires
    
    ## @brief Configures SPI. 
    #  @details Creates an SPI object for SPI between the motors and microcontroller.    
    #
    spi = SPI(2, SPI.MASTER, baudrate=100000, polarity=1, phase=1)
    
    # Creating motor objects.
    
    ## @brief Creates a motor object for the first motor.  
    #  @details Configures the first motor.
    #
    motor1 = stepperdriver.StepperDriver(nCS1, ENN1, spi, reverse=False)
    
    ## @brief Creates a motor object for the second motor.  
    #  @details Configures the second motor.
    #
    motor2 = stepperdriver.StepperDriver(nCS2, ENN2, spi, reverse=False)
    
    # Tell motors where they are starting.  
    motor1.set_zero(3*4*math.pi/9)
    motor2.set_zero(110*2*math.pi/8)
    
    # Creating pen object.
    
    ## @brief Creates a pen object for the linear actuator.   
    #  @details Configures the linear actuator.  
    #
    pen = actuator.Actuator()
    pen.pen_up()
    
    ## @brief Number of motor angles.   
    #  @details Variable containing the number of positions.  
    #
    num = startup()
    
    ## @brief A share containing the number of positions.  
    #  @details This share is used to determine when to cease writing motor angles to the motors.  
    #
    elements = task_share.Share ('h', thread_protect = False, name = "Elements")
    
    elements.put(num)
    
    ## @brief Creates a task for the main function.   
    #  @details This task is called by the scheduler.  
    #
    task1 = cotask.Task(task_main, name='Task_Main', priority=1, period=5, profile=True, trace=False)
    cotask.task_list.append(task1)
    
    gc.collect()
    
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break