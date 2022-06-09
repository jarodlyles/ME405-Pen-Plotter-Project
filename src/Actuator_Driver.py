'''!@file       Actuator_Driver.py
    @brief      A driver for motor control using a duty cycle input.
    @details    This class defines a motor object that can be controlled with a duty cycle (PWM signal).
                In our application, this class is used to drive a linear actuator that uses a DC motor,
                gearboax, and lead screw to provide rotational-to-linear actuator tranduction. As such,
                the class and its functions refer to the functionality of the linear actuator. The X-NUCLEO-IHM04A1 
                motor driver expansion board and the Nucleo L476RG microcontroller are used to control the linear actuator.
                The linear actuator connects to the pen of our plotter, so functions such as down() and up()
                refer to the draw state of the pen.
    @author     Jarod Lyles
    @author     Logan Williamson
    @date       June 1, 2022
'''
from pyb import Pin, Timer, delay

class Actuator:
    '''!@brief      A motor class for one channel.
        @details    Class can be used to apply PWM to a
                    DC motor.
    '''
    def __init__(self):
        '''!@brief           Initializes an object associated with a DC Motor actuator.
            @details         Instantiates a motor object of this class and its relevant attributes.
        '''
        # Please refer to the data sheets for the X-NUCLEO-IHM04A1 and the Nucleo L476RG
        # Enables chip A10 on the board. This pin is needed to able the enable the expansion board.
        self.ENN = Pin(Pin.cpu.A10)
        self.ENN.high()
        
        # Configures output on GPIO pins B4 and B5
        self.PB5 = Pin(Pin.cpu.B5, mode = Pin.OUT_PP)
        self.PB4 = Pin(Pin.cpu.B4, mode = Pin.OUT_PP)
        
        # Sets up the timer and associated PWN channels on pins B4 and B5 for driving the actuator
        # in forward and reverse.
        self.tim3 = Timer(3, freq = 20_000)
        self.t3ch1 = self.tim3.channel(1, Timer.PWM, pin = self.PB4)
        self.t3ch2 = self.tim3.channel(2, Timer.PWM, pin = self.PB5)
    
    def enable(self):
        '''!@brief           Enable the actuator drivers.
            @details         Sets the enable pin high to ready the drivers of the actuator.
        '''
        self.ENN.high()
        
    def disable(self):
        '''!@brief           Disable the actuator drivers.
            @details         Sets the enable pin low to disable the drivers of the actuator.
        '''
        self.ENN.low()
    
    def down(self):
        '''!@brief           Extends the linear actuator (moves the pen down).
            @details         If configured appropreiately with the pins on the expansion board, this function will
                             configure the PWM signal to the motor such that the linear actuator extends at max speed.
        '''
        self.t3ch1.pulse_width_percent(abs(100))
        self.t3ch2.pulse_width_percent(abs(0))

    def up(self):
        '''!@brief           Retracts the linear actuator (moves the pen up).
            @details         If configured appropreiately with the pins on the expansion board, this function will
                             configure the PWM signal to the motor such that the linear actuator extends at max speed.
        '''
        self.t3ch1.pulse_width_percent(abs(0))
        self.t3ch2.pulse_width_percent(abs(100))
    
    
if __name__ == '__main__':
    Pen = Actuator()
    Pen.enable()
    Pen.up()
    #delay(5000)
    #Pen.up()
    #Pen.disable()
    
    