'''!@file       Stepper_Driver.py
    @brief      A driver for performing positional control of a stepper motor using the TMC2208 and TMC4210 drivers.
    @details    This file contains a class that defines a stepper driver object that can control the position
                of a stepper motor. This driver interfaces with the TMC4210 and TMC2208 chips to perform 
                position control of a stepper motor. This driver is designed specifically for positional control of a stepper
                motor.
    @author     Jarod Lyles
    @author     Logan Williamson
    @date       June 1, 2022
'''

from pyb import SPI, Timer, Pin
from time import sleep

class StepperDriver:
    '''!@brief      A class file for interfacing with TMC4210 stepper drivers
        @details    An instance of this class enables the user to interact with the basic 
                parameters required to enable and move a stepper motor driven by a Trinamic 
                TMC4210 stepper motor driver.
    '''

    def __init__(self, chip_select, chip_enable, serial_bus, clock):
        '''
        !@brief             Initializes and returns an object corresponding to a single Trinamic 
                            TMC4210 stepper motor driver.
        @details            This method creates an instance of a TMC4210 stepper driver for 
                            control of a single stepper motor.
        @param chip_select  pyb.Pin object instantiated for an MCU GPIO pin in OUT_PP mode. 
                            Acts as active-low chip select for determining which device is 
                            being communicated with over the SPI bus.
        @param chip_enable  pyb.Pin object instantiated for an MCU GPIO pin in OUT_PP mode.
                            Acts as an active-low enable input for the TMC2208 being controlled by 
                            the TMC4210.
        @param serial_bus   pyb.SPI serial bus object. Used for serial-peripheral communication 
                            between the MCU and the TMC4210.
        @param clock        pyb.Timer object instantiated with period=3, prescaler=4, mode=PWM, pulse_width=2)
                            Used by the TMC4210 for calculation of acceleration ramping functions.
        '''
        #TMC 4210 Register Addresses
        self.en_sd = 0b0110100
        self.lp_REFCONF_RM = 0b0001010
        self.X_TARGET = 0b0000000
        self.X_ACTUAL = 0b0000001
        self.V_MIN    = 0b0000010
        self.V_MAX    = 0b0000011
        self.V_TARGET = 0b0000100 #Probably won't need this, only for constant velocity operating mode
        self.V_ACTUAL = 0b0000101
        self.A_MAX    = 0b0000110
        self.A_ACTUAL = 0b0000111
        self.PMUL_PDIV = 0b0001001
        self.PULSE_RAMP_DIV = 0b0001100
        self.TYPE_VERSION = 0b01110011
        self.send_buf = bytearray(4)
        self.recv_buf = bytearray(4)
        # Movement parameters
        self.ramp_div = 8
        self.pulse_div = 8
        
        #Enable and Chip Select Pin Configuration
        self.ENN = Pin(chip_enable,mode=Pin.OUT_PP)
        self.CS = Pin(chip_select,mode=Pin.OUT_PP)
        
        #SPI Bus Configuration
        self.spi = serial_bus
        self.CLK = clock

        # Set ENN pins high in preparation for instantiating SPI object
        self.ENN.low()  #The 2208 is active low; enable on startup
        self.CS.high()  #Active low; only activate when communicating with chip
        
        # Initialize acceleration ramp parameters to Nonetype
        self.PMUL = None
        self.PDIV = None
        
# =====================================================================
## configureTMC4210()
# =====================================================================
        # Write en_sd (enable step-direction) register to enable driver
        self.send_buf = bytearray([0b01101000,
                                   0b00000000,
                                   0b00000000,
                                   0b00100000])
        self.CS.low()
        self.spi.send_recv(self.send_buf,self.recv_buf)
        self.CS.high()
        
        # Write V_MIN register to set minimum velocity
        self.send_buf = bytearray([0b00000100,
                                   0b00000000,
                                   0b00000000,
                                   0b00000001])
        self.CS.low()
        self.spi.send_recv(self.send_buf,self.recv_buf)
        self.CS.high()
        
        # Write V_MAX register to set maximum velocity
        self.send_buf = bytearray([0b00000110,
                                   0b00000000,
                                   0b00000101,
                                   0b00001000])
        self.CS.low()
        self.spi.send_recv(self.send_buf,self.recv_buf)
        self.CS.high()
        
        # Write PULSE_DIV and RAMP_DIV register to set timing parameters
        self.send_buf = bytearray([0b00011000,
                                   0b00000000,
                                   self.pulse_div<<4|self.ramp_div, # First 4 bits are PULSE_DIV, last 4 are RAMP_DIV. Need RAMP_DIV >= PULSE_DIV-1
                                   0b00000000])
        self.CS.low()
        self.spi.send_recv(self.send_buf,self.recv_buf)
        self.CS.high()
        
        #Write RM register to set ramp mode to default
        self.send_buf = bytearray([0b00010100,
                                   0b00000000,
                                   0b00000000,
                                   0b00000000])
        self.CS.low()
        self.spi.send_recv(self.send_buf, self.recv_buf)
        self.CS.high()
        
        #Write A_MAX register to set maximum acceleration
        self.accel = 0b11111111
        self.send_buf = bytearray([0b00001100,
                                   0b00000000,
                                   0b00000000,
                                   0b11111111])
        self.CS.low()
        self.spi.send_recv(self.send_buf, self.recv_buf)
        self.CS.high()
        
        #Calculate and write PMUL and PDIV to their shared register
        self.pmul_pdiv_vals = StepperDriver.PMUL_PDIV(0b11111111,self.ramp_div,self.pulse_div)
        self.send_buf = bytearray([self.PMUL_PDIV<<1,
                                   0b00000000,
                                   1<<7|self.pmul_pdiv_vals[0],
                                   self.pmul_pdiv_vals[1]])
        self.CS.low()
        self.spi.send_recv(self.send_buf, self.recv_buf)
        self.CS.high()
        
        address = None
        
    def getPosition(self):
        '''
        !@brief     Reads X_ACTUAL register.
        @details    Returns the actual, current position of the stepper motor being 
                    controlled by the TMC4210.
        '''
        self.position = self.readRegister(self.X_ACTUAL)
        return self.position
    
    def getTarget(self):
        '''
        !@brief     Reads X_TARGET register.
        @details    Returns the current target/command position of the stepper motor 
                    being controlled by the TMC4210.
        '''
        self.target = self.readRegister(self.X_TARGET)
        return self.target
        
    def readRegister(self,address):
        '''
        !@brief         Reads the TMC4210 data register associated with the input address.
        @details        Method takes in a 7-bit TMC4210 register address, appends the read 
                        bit to the end of the address, places it in a byte array, and communicates that 
                        array to the device over the SPI bus.
        @param address  7-bit TMC4210 register address (integer equivalent also acceptable)
        '''
        byte_1 = address<<1|1
        self.send_buf = bytearray([byte_1,
                                   0b00000000,
                                   0b00000000,
                                   0b00000000])
        self.CS.low()
        self.spi.send_recv(self.send_buf, self.recv_buf)
        self.CS.high()
        return self.recv_buf
        
    def PMUL_PDIV(accel,RAMP_DIV,PULSE_DIV):
        '''
        !@brief             Calculates all valid PMUL, PDIV pairs based on the current AMAX, 
                            RAMP_DIV, and PULSE_DIV settings. Returns the first valid pair.
        @details            This method is used to calculated a (non-optimized) valid pair of 
                            the ramping function parameters PMUL and PDIV.
        @param accel        AMAX setting of the TMC4210
        @param RAMP_DIV     Ramp divisor ramping function parameter.
        @param PULSE_DIV    Clock divisor ramping function parameter.
        '''
        p = accel/((128)*(2)**(RAMP_DIV-PULSE_DIV))
        pmul_pdiv_list = []
        for pmul in range(128,255):
            for j in range(3,16):
                pdiv = 2**j
                p_prime = pmul/pdiv
                q = p_prime/p
                if 0.95 <= q and q <= 1.0:
                    PMUL = pmul
                    PDIV = j-3
                    pmul_pdiv_list.append((PMUL,PDIV))
        # print(pmul_pdiv_list)
        return pmul_pdiv_list.pop(0)
    
    def setPosition(self,position):
        '''
        !@brief         Sets the target step/position by writing to the X_TARGET register address.
        @details        Takes an (unsigned) integer value between 0 and 16,777,215 and converts it  
                        to a bytearray format, then writes it to the appropriate X_TARGET register 
                        address using the SPI bus.
        @param position Unsigned integer value from 0 to 16,777,215 (up to 3 bytes binary representation)
        '''
        MSB = position>>16
        mSB = (position>>8)&0b0000000011111111
        LSB = position&0b000000000000000011111111
        self.send_buf = bytearray([self.X_TARGET<<1,
                                   MSB,
                                   mSB,
                                   LSB])
        self.CS.low()
        self.spi.send_recv(self.send_buf, self.recv_buf)
        self.CS.high()
        
    
        

if __name__=='__main__':
    
    # Timer signal setup
    timer = Timer(4, period=3, prescaler=0)
    CLK = timer.channel(1, pin=Pin.cpu.B6, mode=Timer.PWM, pulse_width=2)
    # Set up SPI serial communication
    spi = SPI(2, SPI.CONTROLLER, baudrate = 1_000_000, polarity = 1, phase = 1)
    # Driver object(s) instantiation
    DRV1 = StepperDriver(Pin.cpu.B0,Pin.cpu.C3,spi,CLK)
    DRV2 = StepperDriver(Pin.cpu.C0,Pin.cpu.C2,spi,CLK)
    
#     DRV1.PMUL_PDIV(64,1,1)
#Check the TYPE_VERSION readout from the TMC4210
#Should return b2b1b0 = 429101
    type_buf = bytearray([0b01110011,
                          0b00000000,
                          0b00000000,
                          0b00000000])
    buf = bytearray(4)
    DRV1.ENN.low()
    DRV1.CS.low()
    DRV1.spi.send_recv(type_buf,buf)
    DRV1.CS.high()
    for idx,byte in enumerate(buf): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
    print('^TYPE_VERSION readout \n')

##__init__ register writes testing/reading; seem to be working
    #Check that EN_SD register write went through
    DRV1.readRegister(DRV1.en_sd)
    for idx,byte in enumerate(DRV1.recv_buf): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
    print('^EN_SD readout \n')
    
    #Check the values written to V_MIN and V_MAX
    DRV1.readRegister(DRV1.V_MIN)
    for idx,byte in enumerate(DRV1.recv_buf): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
    print('^V_MIN readout \n')
    
    DRV1.readRegister(DRV1.V_MAX)
    for idx,byte in enumerate(DRV1.recv_buf): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
    print('^V_MAX readout \n')
    
    DRV1.readRegister(DRV1.PULSE_RAMP_DIV)
    for idx,byte in enumerate(DRV1.recv_buf): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
    print('^PULSE_DIV and RAMP_DIV readout \n')
    
    DRV1.readRegister(DRV1.PMUL_PDIV)
    for idx,byte in enumerate(DRV1.recv_buf): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
    print('^PMUL_PDIV readout \n')
    
    DRV1.setPosition(1600)
    DRV2.setPosition(0)  