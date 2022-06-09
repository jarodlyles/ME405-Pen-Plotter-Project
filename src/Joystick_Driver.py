'''!@file       Joystick_Driver.py
    @brief      A driver for joystick analog read.
    @details    This class defines a joystick object that can read the analog outputs of potentiometers of
                a joystick controller using on-board ADC input pins. The Nucleo L476RG microcontroller
                is used to read from the joystick potentiometers. The push-button switch of the joystick
                can also be read. This class is specifically designed  for use case of manual control of
                a pen plotter mechanism.
    @author     Jarod Lyles
    @author     Logan Williamson
    @date       June 1, 2022
'''

from pyb import Pin, Timer, delay, ADC

class Joystick:
    '''!@brief      A joystick class to read one joystick.
        @details    Class can be used to setup a joystick on a Nucleo l476RG board.
    '''
    def __init__(self):
        '''!@brief           Initializes an object associated with a joystick controller.
            @details         Instantiates a joystick object of this class and its relevant attributes.
        '''
        # Sets up analog pins on the L476RG for the joystick potentiometers
        self.PA0 = Pin(Pin.cpu.A0, mode=Pin.IN)
        self.PA1 = Pin(Pin.cpu.A1, mode=Pin.IN)
        
        # Sets up a digital pin on the L476RG for pull up switch input
        self.PA8 = Pin(Pin.cpu.A8, mode=Pin.IN, pull=Pin.PULL_UP)
        
        # Sets up a digital output on the L476RG for controlling an LED
        self.LED = Pin(Pin.cpu.B10, mode=Pin.OUT_PP)
        
        # Assigns the GPIO pin to 
        self.SW = self.PA8
        
        # Sets up the ADC read for the potentiometers.
        self.VRX = ADC(self.PA0)
        self.VRY = ADC(self.PA1)
        
        # Sets a boolean to monitor toggle of the pen.
        self.PEN_TOGGLE = False
        
    def vrx_read(self):
        '''!@brief           Reads the X-position potentiometer of the joystick.
            @details         Uses the 12-bit ADC of the L476RG to read the X-position potentiometer of the
                             joystick. A conditional handles any bias from zero position of the potentiometer
                             when released.
        '''
        vrx = 0
        if abs(self.VRX.read() - 2048) > 100:
            vrx = self.VRX.read() - 2048
        return vrx
        
    def vry_read(self):
        '''!@brief           Reads the Y-position potentiometer of the joystick.
            @details         Uses the 12-bit ADC of the L476RG to read the Y-position potentiometer of the
                             joystick. A conditional handles any bias from zero position of the potentiometer
                             when released.
        '''
        vry = 0
        if abs(self.VRY.read() - 2048) > 100:
            vry = self.VRY.read() - 2048
        return vry
    
    def sw_read(self):
        '''!@brief           Reads the switch position of the joystick.
            @details         Uses the pull-up enabled GPIO input to read the switch condition of the joystick's
                             push button.
        '''
        return not self.SW.value()
    
    def toggle_pen(self):
        '''!@brief           Set an LED on or off to indicate toggle of the joystick switch.
            @details         Uses a digital pin of the L476RG to turn an LED on or off with a toggle
                             logic control. This was useful for testing use the switch of the joystick
                             to toggle up and down the pen actuator of the plotter in manual mode.
        '''
        if self.sw_read():
            self.PEN_TOGGLE = not self.PEN_TOGGLE
            delay(200)
        
        if self.PEN_TOGGLE:
            self.LED.high()
        elif not self.PEN_TOGGLE:
            self.LED.low()
            
        return self.PEN_TOGGLE


if __name__ == '__main__':
    
    JS = Joystick()
    while True:
        print(JS.vrx_read())
        print(JS.vry_read())
        #print(JS.SW_read())
        delay(10)