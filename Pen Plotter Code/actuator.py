'''!
    @file       actuator.py
    
    @brief      Driver to be used to interact with the 12 V Linear Actuator.
    
    @details    This driver manipulates the pins that are connected to the X_Nucleo IHM04A1 DC Motor Breakout Shield.

                
    @author     Jake Lesher
    @author     Baxter Bartlett
    @date       6/5/2022
    
'''
import utime
from pyb import Pin

class Actuator:
    '''!@brief      Driver to be used to interact with the 12 V Linear Actuator.
        @details    This driver manipulates the pins that are connected to the X_Nucleo IHM04A1 DC Motor Breakout Shield.
    '''
    
    def __init__(self):
        '''!@brief   Configures the Linear Actuator.
            @details Configures all the pins on the Nucleo such that the Nucleo can control the Linear Actuator.  
        '''
        EN_A_PIN = Pin.cpu.A10
        IN1A_PIN = Pin.cpu.B4
        IN2A_PIN = Pin.cpu.B5
        
        # EN_B_PIN = Pin.cpu.C1
        # IN1B_PIN = Pin.cpu.A0
        # IN2B_PIN = Pin.cpu.A1
        
        self.EN_A = Pin(EN_A_PIN, mode=Pin.OUT_PP, value=0)
        self.IN1A = Pin(IN1A_PIN, mode=Pin.OUT_PP, value=0)
        self.IN2A = Pin(IN2A_PIN, mode=Pin.OUT_PP, value=0)
        
    def pen_down(self):
        '''!@brief   Places the pen down.
            @details This method allows the user to tell the actuator to place the pen on the paper.  
        '''
        self.EN_A.high()
        self.IN2A.high()
        self.IN1A.low()
        utime.sleep(0.2)
        self.IN2A.low()
        self.lock_pen()
    
    def pen_up(self):
        '''!@brief   Places the pen up.
            @details This method allows the user to tell the actuator to take the pen off the paper.  
        '''
        self.EN_A.high()
        self.IN1A.high()
        self.IN2A.low()
        utime.sleep(0.2)
        self.IN1A.low()
        self.lock_pen()
        
    def lock_pen(self):
        '''!@brief   Locks pen in place.  
            @details Once a pen position is reached, this method helps lock the pen in place.  
        '''
        self.EN_A.high()
        self.IN1A.high()
        self.IN2A.high()

# if __name__ == '__main__':
    
#     pen = Actuator()
#     pen.pen_up()
#     utime.sleep(1)
#     pen.pen_down()
#     utime.sleep(1)
    
#     pen.pen_up()
#     utime.sleep(1)
#     pen.pen_down()
#     utime.sleep(1)
#     pen.pen_up()