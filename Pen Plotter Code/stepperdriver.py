'''!
    @file       stepperdriver.py
    
    @brief      Driver to be used with the TMC4210 and TMC2208 IC's.
    
    @details    This driver enables control over a stepper motor by interfacing with both the TMC 2208 
                and the TMC 4210 (as control over both is required to drive a stepper motor).  

                
    @author     Jake Lesher
    @author     Baxter Bartlett
    @date       6/5/2022
    
'''
import utime, math

class StepperDriver:
    '''!@brief      Driver to be used with the TMC4210 and TMC2208 IC's.
        @details    This driver enables control over a stepper motor by interfacing with both the TMC 2208 
                    and the TMC 4210 (as control over both is required to drive a stepper motor).  
    '''
    
    def __init__(self, nCS, ENN, spi, reverse):
        '''!@brief   Creates a motor object by asking for an enable pin, a chip select pin, and an SPI object.  
                     Also requires a Boolean specification to switch which rotational direction is positive.  
            @details After receiving all the pins, the registers of the TMC 4210 are assigned to variables.  From there, 
                     the class puts the motors through an initialization process specified by the manual.  Lastly, TX and RX 
                     buffers are created for sending/receiving data.  
        '''
        self.spi = spi
        self.nCS = nCS
        self.ENN = ENN
        
        # Defining the useful register addresses. 
        self.X_TARGET = 0b0000000
        self.X_ACTUAL = 0b0000001
        self.V_TARGET = 0b0000100
        self.A_MAX = 0b0000110
        self.RAMP_MODE = 0b0001010
        self.X_LATCHED = 0b0001110
        self.TYPE_VERSION = 0b0111001
        self.PMUL_PDIV = 0b0001001
        self.IF_CONF = 0b0110100 # Use for inverting step/dir/ref
        self.V_MIN = 0b0000010
        self.V_MAX = 0b0000011
        self.PULSE_RAMP_DIV = 0b0001100
        
        # Defining the bytearray, or datagram, that will be sent over SPI.
        self.TX_buf = bytearray(4)
        self.RX_buf = bytearray(4)
        
        # Step 1: Setting en_sd=1
        self.TX_buf[0] = (self.IF_CONF<< 1)|0 # first 8 digits of datagram
        self.TX_buf[1] = 0b00000000 
        self.TX_buf[2] = 0b00000000 
        if reverse == True:
            self.TX_buf[3] = 0b00110000
        else:
            self.TX_buf[3] = 0b00100000
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()
        
        # Step 2: Setting V_MIN and V_MAX
        self.set_vmin(20)
        self.set_vmax(400)
        
        # Step 3: Setting PULSE_DIV and RAMP_DIV
        self.set_divs(9, 9)
        
        # Step 4: Setting A_MAX
        self.set_accel(100)
        
        # Step 5: Setting Ramp Mode
        self.set_mode("ramp")
        
        # Step 6: Choose reference switch configuration.

    def test_version(self):
        '''!@brief   Allows testing of whether motor is successfuly configured.  
            @details This method reads from the TYPE_VERSION register of the TMC 4210.  If the value 0x15429101 
            is read, then the motor is properly configured.  
        '''
        print("Test version:")
        print('Retrieving datagram from TYPE_VERSION register.')
        self.TX_buf[0] = (self.TYPE_VERSION << 1)|1
        self.TX_buf[1] = 0
        self.TX_buf[2] = 0
        self.TX_buf[3] = 0
        #print(f"Sending {self.TX_buf}")
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()
        query = self.RX_buf[0] << 24 | self.RX_buf[1] << 16 | self.RX_buf[2] << 8 | self.RX_buf[3]
        print(f"Received Datagram: {query:#04x}")
    
    def set_mode(self, mode):
        '''!@brief   Allows selection of motor mode. 
            @details The TMC 4210 has multiple motor modes.  By passing in the name of the desired mode, this method puts
                     the motor in that mode.  
        '''
        if mode == "ramp":
            self.TX_buf[0] = (self.RAMP_MODE<< 1)|0 # first 8 digits of datagram
            self.TX_buf[1] = 0b00000000 
            self.TX_buf[2] = 0b00000100 
            self.TX_buf[3] = 0b00000000 # setting to ramp mode
            self.nCS.low()
            self.spi.send_recv(self.TX_buf, self.RX_buf)
            self.nCS.high()
            print("Setting to ramp mode.")
        elif mode == "hold":
            self.TX_buf[0] = (self.RAMP_MODE<< 1)|0 # first 8 digits of datagram
            self.TX_buf[1] = 0b00000000 
            self.TX_buf[2] = 0b00000100 
            self.TX_buf[3] = 0b00000011 # setting to hold mode
            self.nCS.low()
            self.spi.send_recv(self.TX_buf, self.RX_buf)
            self.nCS.high()
            print("Setting to hold mode.")
        elif mode == "velocity":
            self.TX_buf[0] = (self.RAMP_MODE<< 1)|0 # first 8 digits of datagram
            self.TX_buf[1] = 0b00000000 
            self.TX_buf[2] = 0b00000100 
            self.TX_buf[3] = 0b00000010 # setting to velocity mode
            self.nCS.low()
            self.spi.send_recv(self.TX_buf, self.RX_buf)
            self.nCS.high()
            print("Setting to velocity mode.")
        elif mode == "soft":
            self.TX_buf[0] = (self.RAMP_MODE<< 1)|0 # first 8 digits of datagram
            self.TX_buf[1] = 0b00000000 
            self.TX_buf[2] = 0b00000100 
            self.TX_buf[3] = 0b00000001 # setting to soft mode
            self.nCS.low()
            self.spi.send_recv(self.TX_buf, self.RX_buf)
            self.nCS.high()
            print("Setting to soft mode.")
        else:
            print("Invalid mode.")
            
    def set_divs(self, pulsediv, rampdiv):
        '''!@brief   Writes a pulse_div and a ramp_div to the TMC 4210.  
            @details The TMC 4210 requires the user to choose a pulse_div and a ramp_div in order to control the motors.  
                     This method allows the user to specify a pulse_div and ramp_div.  
        '''
        self.TX_buf[0] = (self.PULSE_RAMP_DIV<< 1)|0 # first 8 digits of datagram
        self.TX_buf[1] = 0b00000000 
        self.TX_buf[2] = ((pulsediv & 0b1111) << 4) | (rampdiv & 0b1111)
        self.TX_buf[3] = 0b00000000 
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()
    
    def set_vmin(self, vmin):
        '''!@brief   Writes a minimum velocity to the TMC 4210.   
            @details The TMC 4210 requires that a minimum velocity for the motor be specified.  This method allows for the 
                     user to specify one.  
        '''
        self.TX_buf[0] = (self.V_MIN<< 1)|0 # first 8 digits of datagram
        self.TX_buf[1] = 0b00000000 
        self.TX_buf[2] = (vmin >> 8) & 0b111 
        self.TX_buf[3] = vmin & 0xFF 
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()
        
    def set_vmax(self, vmax):
        '''!@brief   Writes a maximum velocity to the TMC 4210.   
            @details The TMC 4210 requires that a maximum velocity for the motor be specified.  This method allows for the 
                     user to specify one.  
        '''
        self.TX_buf[0] = (self.V_MAX<< 1)|0 # first 8 digits of datagram
        self.TX_buf[1] = 0b00000000 
        self.TX_buf[2] = (vmax >> 8) & 0b111 
        self.TX_buf[3] = vmax & 0xFF 
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()
    
    def set_velocity(self, target_vel):
        '''!@brief   Allows speed control of the stepper motor.
            @details The TMC 4210 allows the user to specify a desired speed at which to spin the motor.  This method 
                     allows the user to pass in a desired velocity.  From there, the method places the stepper motor 
                     in velocity mode (the mode necessary for speed control) and writes the desired speed to the corresponding 
                     register.  
        '''
        self.set_mode("velocity")
        
        self.TX_buf[0] = (self.V_TARGET << 1)|0 # first 8 digits of datagram
        self.TX_buf[1] = 0b00000000 # second 8 digits of datagram
        
        # then, append target_vel as a 12 bit signed number
        if target_vel < 0:
            target_vel += 2**12

        self.TX_buf[2] = (target_vel >> 8) & 0b1111
        self.TX_buf[3] = (target_vel)
        
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()

        
    def set_accel(self, max_accel):
        '''!@brief   Allows specification of deceleration ramp.  
            @details In order to control the position of a stepper motor, the TMC 4210 requires that a maximum acceleration for the 
                     deceleration ramp (the ramp characteristic of the motor slowing down once near its desired location) be specified.  
                     Two scaling factors, PMUL and PDIV, are also required.  PMUL can be any integer between 128 and 255 while PDIV 
                     can be any integer between 0 and 13.  In order to be valid the following relation must be satisfied:
                         
                         0.95 < (PMUL/AMAX)*2^(4-PDIV) < 1
                         
                     Thus, for a specified maximum acceleration, this method runs through all possible PMUL and PDIV combinations until 
                     a valid combination is found.  Then all three values are written to their respective registers.  
        '''
        pd = range(14)
        pm = range(128,256)
        pmul = 128
        valid = 0
        while pmul<len(pm)+128:
            pdiv = 0
            while pdiv<len(pd):
                q = (pmul/max_accel)*2**(4-pdiv)
                if q>0.95 and q<1:
                    valid = 1
                    break
                else:
                    pdiv+=1
            if valid == 1:
                break
            else:
                pmul+=1
        
        self.TX_buf[0] = (self.PMUL_PDIV<< 1)|0 # first 8 digits of datagram
        self.TX_buf[1] = 0b00000000 # second 8 digits of datagram
        self.TX_buf[2] = 0b10000000 | (pmul & 0b01111111)
        self.TX_buf[3] = (pdiv & 0b00001111)
        
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()
        
        self.TX_buf[0] = (self.A_MAX << 1)|0
        self.TX_buf[1] = 0b00000000
        self.TX_buf[2] = (max_accel >> 8) & 0b111
        self.TX_buf[3] = (max_accel & 0b11111111)
        
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()


    def set_target_pos(self, target_pos):
        '''!@brief   Allows specification of a desired position. 
            @details Accepts a desired stepper motor position and writes it to the corresponding register.  
        '''
        target_pos *= (8*48)/(2*math.pi) # input in rad, output in usteps
        target_pos = int(target_pos)
        self.TX_buf[0] = (self.X_TARGET<< 1)|0 # first 8 digits of datagram
        self.TX_buf[1] = (target_pos >> 16) & 0xFF
        self.TX_buf[2] = (target_pos >> 8) & 0xFF
        self.TX_buf[3] = target_pos & 0xFF
        
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()
        
    def set_zero(self, offset):
        '''!@brief   Allows user to specify motor starting position.  
            @details Allows the user to specify the starting position of the motor if it is imperative that the motor's starting 
                     position not be zero microsteps.  To do so, the motor is temporarily placed into hold mode (the mode required 
                     to set a motor's location), and then the specified location is written to the corrsponding register.  
        '''
        self.set_mode("hold")
        utime.sleep(0.2)
        
        offset *= (8*48)/(2*math.pi) # input in rad, output in usteps
        offset = int(offset)
        self.TX_buf[0] = (self.X_ACTUAL<< 1)|0 # first 8 digits of datagram = register+write access
        self.TX_buf[1] = (offset >> 16) & 0xFF
        self.TX_buf[2] = (offset >> 8) & 0xFF
        self.TX_buf[3] = offset & 0xFF
        
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()
        
        utime.sleep(0.2)
        self.set_mode("ramp")

    # def get_zero(self):
    #     '''!@brief   Creates a motor object by asking for an enable pin, a chip select pin, and an SPI object.  
    #                  Also requires a Boolean specification to switch which rotational direction is positive.  
    #         @details After receiving all the pins, the registers of the TMC 4210 are assigned to variables.  From there, 
    #                  the class puts the motors through an initialization process specified by the manual.  Lastly, TX and RX 
    #                  buffers are created for sending/receiving data.  
    #     '''
    #     self.TX_buf[0] = (self.X_LATCHED<< 1)|1 # first 8 digits of datagram = register+read access
    #     self.TX_buf[1] = 0
    #     self.TX_buf[2] = 0
    #     self.TX_buf[3] = 0
        
    #     self.nCS.low()
    #     self.spi.send_recv(self.TX_buf, self.RX_buf)
    #     self.nCS.high()
        
    #     last_actual_zero = (self.RX_buf[1]<<16) | (self.RX_buf[2]<<8) | (self.RX_buf[3])
        
    #     return last_actual_zero
        
    def get_target_pos(self):
        '''!@brief   Returns the target position of the motor.
            @details Reads from the corresponding register and returns the position that the motor is targeted for.   
        '''
        self.TX_buf[0] = (self.X_TARGET<< 1)|1 # first 8 digits of datagram = register+read access
        self.TX_buf[1] = 0
        self.TX_buf[2] = 0
        self.TX_buf[3] = 0
        
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()
        
        target_pos = (self.RX_buf[1]<<16) | (self.RX_buf[2]<<8) | (self.RX_buf[3])
        target_pos *= (2*math.pi)/(8*48)
        
        return target_pos # in radians
    
    def arrived(self, tolerance):
        '''!@brief   Determines whether the motor has arrived to its target position.   
            @details Computes the difference between the target position and actual position.  If the distance is greater than 
                     a specified tolerance, then the method returns a True Boolean.  Otherwise, it returns a False Boolean.  
        '''
        if abs(self.get_pos()-self.get_target_pos()) <= tolerance:
            arrived_bool = True
        else:
            arrived_bool = False
        return arrived_bool
    
    def get_pos(self):
        '''!@brief   Returns the current position of the motor.  
            @details Reads from the corresponding register and returns the current position of the motor.  
        '''
        self.TX_buf[0] = (self.X_ACTUAL<< 1)|1 # first 8 digits of datagram = register+read access
        self.TX_buf[1] = 0
        self.TX_buf[2] = 0
        self.TX_buf[3] = 0
        
        self.nCS.low()
        self.spi.send_recv(self.TX_buf, self.RX_buf)
        self.nCS.high()
        
        actual_pos = (self.RX_buf[1]<<16) | (self.RX_buf[2]<<8) | (self.RX_buf[3])
        actual_pos *= (2*math.pi)/(8*48)
        
        return actual_pos