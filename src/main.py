"""!
@file       main.py
@brief      Manual and automated drawing program for a drawing robot operating 
            in a polar coordinate system.
@details    This file constitutes a cooperative multitasking program which can read a 
            .hpgl file, parse and interpolate the contained command and coordinate data, 
            compute the stepper motor angles required to actuate a plotting machine to 
            those coordinates, and command the motors to actuate to those positions. The
            result is a drawn image of the .hpgl.
@author     Jarod Lyles
@author     Logan Williamson
@date       June 3, 2022
"""

###############################################################################
# Functions called from within multitasking program
###############################################################################
def g(x, theta):
    '''
    !@brief          The function g(x, theta) defines the difference between the value of 
                    the polar-to-Cartesian coordinate transform function, f(theta), 
                    and the desired input Cartesian coordinates x and y.
    @details        This function takes an input Cartesian coordinate pair x and y 
                    contained in input vector x, and a pair of angular coordinates, 
                    theta_1 and theta_2, contained in input vector theta. It evaluates
                    the polar-to-Cartesian coordinate transform of the input values in
                    the theta vector, then subtracts the results element-wise from the
                    x vector input. This function will therefore converge to zero as the
                    input theta vector contents approach appropriate values corresponding 
                    to the x vector contents via this transform. This function can be 
                    passed to an iterative Newton-Raphson solver to iterate for the 
                    input theta vector corresponding to a desired Cartesian coordinate pair.
    @param x        numpy.array object that is a 2x1 column vector containing an x- and
                    y-coordinate pair.
    @param theta    numpy.array object that is a 2x1 column vector containing a theta_1 
                    and theta_2 coordinate pair. The theta_1 coordinate is related to 
                    the radial coordinate in the polar coordinate system by the conversion 
                    factor N, corresponding to the thread pitch of a linear actuator.
    '''
    f_arr = np.array([[N.get()*theta[0][0]*math.cos(theta[1][0])],
                      [N.get()*theta[0][0]*math.sin(theta[1][0])]])
    g = x - f_arr
    return g

def dg_dtheta(theta):
    '''
    !@brief          The function dg_dtheta(theta) computes the partial derivative 
                    of the function g(x,theta).
    @details        This function takes an input pair of angular coordinates, 
                    theta_1 and theta_2, contained in input vector theta. It evaluates
                    the partial derivative of the polar-to-Cartesian coordinate transform 
                    of these input coordinates. The result is the negated Jacobian 
                    transform of the system, to be used in the Newton-Raphson algorithm 
                    to iteratively calcuate the theta vector associated with a given 
                    Cartesian coordinate pair.
    @param theta    numpy.array object that is a 2x1 column vector containing a theta_1 
                    and theta_2 coordinate pair. The theta_1 coordinate is related to 
                    the radial coordinate in the polar coordinate system by the conversion 
                    factor N, corresponding to the thread pitch of a linear actuator.
    '''
    dg_dtheta = np.array([[-N.get()*math.cos(theta[1][0]),N.get()*theta[0][0]*math.sin(theta[1][0])],
                          [-N.get()*math.sin(theta[1][0]),-N.get()*theta[0][0]*math.cos(theta[1][0])]])
    return dg_dtheta

def NewtonRaphson(fcn,jacobian,guess,thresh):
    '''
    !@brief          Newton-Raphson root finding algorithm
    @details        This function carries out iterative solution of the Newton-Raphson
                    method for approximating the nearest root of a function. This method 
                    requires an anonymous input function handle, corresponding to g(x,theta),
                    where the x vector is a static value corresponding to the desired 
                    Cartesian coordinate for a plotting device. The theta vector should be 
                    allowed to vary, such that iterative solution of this algorithm will cause 
                    the output theta vector values to converge to the polar coordinate pair 
                    which will actuate a polar plotting device to the desired Cartesian coordinate.
    @param fcn      An anonymous function handle containing the desired output Cartesian
                    coordinates of a plotting device, and the input guess value for the 
                    associated polar coordinate pair to be varied by the solver.
    @param jacobian The Jacobian of the system being analyzed (which can be found 
                    by passing in the function dg_dtheta(theta).
    @param guess    An initial guess value for the theta vector used as the starting 
                    point for the first solver iteration.
    @param thresh   An error threshold value used to determine if the output error 
                    of the function has reached an adequately close approximation 
                    of the theta value required to yield the static desired output 
                    of the anonymous function.
    '''
    step = 0
    error = math.inf
    theta = guess
    while error > thresh:
        g = fcn(theta)
        error = np.linalg.norm(g)
        theta = theta - np.dot(np.linalg.inv(jacobian(theta)),g)
        step+=1
    return theta

def parse_hpgl(string):
    '''
    !@brief          Hewlitt-Packard Graphics Language file parsing method.
    @details        This method reads a text or .hpgl file containing Hewlitt-Packard
                    Graphics Language plotting device commands and coordinate data. It
                    removes all errant spaces and pen selection commands and therefore 
                    must only be used with single-pen devices. It then parses the useful 
                    pen actuation and coordinate data into [command,x-coordinate,y-coordinate]
                    formatted lists and stores each of these triplet command sets within 
                    a larger list for ease of data handling.
    @param string   A string object containing the entire readout from a .hpgl or .txt 
                    file containing Hewlitt-Packard Graphics Language data.
    '''
    data_list = []
    string = string.replace('SP1;','')
    string = string.replace('SP0;','')
    string = string.replace(' ','')
    string = string.replace('IN;','')
    string = string.replace('PU;','')
    content = string.split(';')
    for i in range(0,len(content)):
        if 'PU' in content[i]:
            sub_list = [0]
            content[i] = content[i].replace('PU','')
            content[i] = content[i].split(',')
            if len(content[i])>2:
                for j in range(0,len(content[i])-1,2):
                    sub_list.append(int(content[i][j])/dpi+offset)
                    sub_list.append(int(content[i][j+1])/dpi+offset)
                    data_list.append(sub_list)
                    sub_list = [0]
            else:
                sub_list.append(int(content[i][0])/dpi+offset)
                sub_list.append(int(content[i][1])/dpi+offset)
                data_list.append(sub_list)
                
        if 'PD' in content[i]:
            sub_list = [1]
            content[i] = content[i].replace('PD','')
            content[i] = content[i].split(',')
            if len(content[i])>2:
                for j in range(0,len(content[i])-1,2):
                    sub_list.append(int(content[i][j])/dpi+offset)
                    sub_list.append(int(content[i][j+1])/dpi+offset)
                    data_list.append(sub_list)
                    sub_list = [1]
            else:
                sub_list.append(int(content[i][0])/dpi+offset)
                sub_list.append(int(content[i][1])/dpi+offset)
                data_list.append(sub_list)
    return data_list

def interpolate(data_list):
    '''
    !@brief              Method for scaled linear interpolation of Cartesian coordinate pairs.
    @details            This function iterates through a list containing lists of the format 
                        [command,x-coordinate,y-coordinate] as is produced by the parse_hpgl()
                        method. For each triplet entry in this data_list object, the method 
                        calculates the distance between the entry and the next entry in the 
                        list and interpolates 100 points per inch of distance between each 
                        existing point. It then writes the resulting interpolated data set
                        to a text file.
    @param data_list    A Python list object containing lists of the form 
                        [command,x-coordinate,y-coordinate] to be interpolated at a point 
                        density of 100 points per inch.
    '''
    xy = open('xy_values.txt','w')
    for i in range(0,len(data_list)-1):
        xy.write(f'{data_list[i][0]},{data_list[i][1]},{data_list[i][2]}\r\n')
        #Calulate number of points to generate between each .hpgl command
        x_dist = data_list[i+1][1] - data_list[i][1]
        y_dist = data_list[i+1][2] - data_list[i][2]
        rss = (x_dist**2+y_dist**2)**0.5
        interpoints = round(rss*100)
        #Write interpolated datapoints to text file
        for j in range(0,interpoints):
            xy.write(f'{data_list[i+1][0]},{data_list[i][1] + x_dist*j/interpoints},{data_list[i][2] + y_dist*j/interpoints}\r\n')
    xy.close()
    return None

def int2bytes(integer):
    '''
    !@brief          Integer to byte-wise conversion for unsigned integers.
    @details        This method converts unsigned integer values 0 to 16,777,215 into the 
                    corresponding 3 bytes required for generation of datagram used in SPI 
                    communication with the TMC4210 stepper driver.
    @param integer  Unsigned integer value between 0 and 16,777,215
    '''
    lsb = integer&0xFF
    midsb = integer>>8&0xFF
    msb = integer>>16&0xFF
    return lsb, midsb, msb
    
def bytes2int(byte_array):
    '''
    !@brief              Bytearray to integer conversion for four byte datagrams.
    @details            This method converts the (unsigned) last three bytes of a four byte 
                        bytearray object into the corresponding integer value. This is used 
                        for converting received datagram buffers from the TMC4210 stepper 
                        driver into decimal form.
    @param byte_array   Unsigned integer value between 0 and 16,777,215
    '''
    integer = byte_array[1]<<16 | byte_array[2]<<8 | byte_array[3]
    return integer

def button_callback(trig_line):
    '''
    !@brief              Button callback external interrupt.
    @details            This function is called in the external interrupt to count button presses
                        and invert the flag representing the button press state. It follows the
                        direction of documentation to accept one argument that is the trigger line.  
    '''
    button_flag.put(not button_flag.get())
###############################################################################
# Cooperative Multitasking Generator Functions
###############################################################################

def generate_thetas():
    '''
    !@brief      Generator function for reading Cartesian coordinates from a text 
                file and calculating the associated polar coordinate pairs.
    @details    The generate_thetas() method is a generator function called by the 
                cotask.py task manager. It opens and reads  parsed Hewlitt-Packard 
                Graphics Language command and coordinate data from the xy_values.txt 
                file created by the interpolate() method. It parses each coordinate 
                pair and calls the NewtonRaphson() method to approximate the associated 
                polar coordinates. These coordinates and the associated command data are
                then written to task_share.py queues to be read by the 
                actuate_motors() method.
    '''
    theta_guess = np.array([[1],[1]])       #Initial guess for Newton-Raphson
    xy = open('xy_values.txt','r')
    xy_data = xy.readline().strip('\r\n').split(',')
    while True:
        if xy_data[0] != '':
            cmd = float(xy_data[0])
            x = float(xy_data[1])
            y = float(xy_data[2])
            x_des = np.array([[x],[y]])
            theta = NewtonRaphson(lambda theta:g(x_des,theta),dg_dtheta,theta_guess,1e-6)
            theta_guess = theta
            if not cmd_queue.full():
                cmd_queue.put(cmd)
                theta_r.put(theta[0][0])
                theta_t.put(theta[1][0])
                xy_data = xy.readline().strip('\r\n').split(',')
            elif cmd_queue.full():
                pass
            #th.write(f'{xy_data[0]},{theta[0][0]},{theta[1][0]}\r\n')
        elif xy_data[0] == '':
            xy.close()
        yield None
            
def actuate_motors():
    '''
    !@brief      Generator function for reading polar coordinates and writing these 
                coordinates to the appropriate motor driver object for actuation of 
                a polar plotting mechanism.
    @details    The actuate_motors() method is a generator function called by the 
                cotask.py task manager. This task operates as a finite state machine 
                containing four mutually exclusive operating states. These include: an 
                initialization state (0). A manual jog state (1) which uses an MCU 
                ADC pin to read potentiometer inputs from a joystick used to control 
                manual jogging of the turntable and linear actuator of the polar plotting 
                machine. A plotter position checking state (2) which reads from each stepper 
                motor driver what the current target position and actual position are, 
                computes the difference, and compares the magnitude of the difference to 
                an error threshold to determine whether the motors are ready to receive 
                an additional command yet. Finally, an autodrawing state which reads the 
                next motor position command from the task_share.py queues and writes the 
                command to the stepper  motor driver.
                
    '''
    state = 0
    while True:
    # State 0 - Initialization State
        if state == 0:
            print(f'Motor State is: {state}')
            threshold = 5
            rad2step = 1600/(2*3.14159)
            #state = 0
            position1 = 0
            position2 = 0
            pos1_div = 15
            pos2_div = 60
            radial_lim = 40000000
            state = 1
            
    # State 1 - Manual Jog State
        elif state == 1:
            print(f'Motor State is: {state}')
            Pen.up()
            if Joystick.toggle_pen():
                Pen.down()
            elif not Joystick.toggle_pen():
                Pen.up()
            if position1 >= radial_lim:
                position1 = radial_lim - 1
            elif position1 <= -radial_lim:
                position1 = -radial_lim + 1
            else:
                position1 += Joystick.vrx_read()
            position2 += Joystick.vry_read()
            DRV1.setPosition(round(position1/pos1_div))
            DRV2.setPosition(round(position2/pos2_div))
            if button_flag.get() == False:
                #th = open('theta_values.txt','r')
                state = 2
            
    # State 2 - Check Position
        elif state == 2:
            print(f'Motor State is: {state}')
          # Read X_TARGET and X_ACTUAL, compare, and if equal, read and send a new command
            diff_drv1 = bytes2int(DRV1.getPosition()) - bytes2int(DRV1.getTarget())
            diff_drv2 = bytes2int(DRV2.getPosition()) - bytes2int(DRV2.getTarget())
            if  abs(diff_drv1)<threshold and abs(diff_drv2)<threshold:
                state = 3
    # State 3 - Autodraw
        elif state == 3:
            command = [0,0,0]
            print(f'Motor State is: {state}')
            if cmd_queue.any():
                command[0] = cmd_queue.get()
                command[1] = theta_r.get()
                command[2] = theta_t.get()
                rad_theta = round((float(command[1]))*rad2step)
                turn_theta = round((float(command[2]))*rad2step)
                if round(float(command[0])) == 1:
                    Pen.down()
                elif round(float(command[0])) == 0:
                    Pen.up()
                DRV1.setPosition(rad_theta)
                DRV2.setPosition(turn_theta)
            if cmd_queue.empty():
                state = 1
        yield state
    
if __name__ == '__main__':
    import os
    import cotask
    import task_share
    import array
    import gc
    from ulab import numpy as np
    import math
    from pyb import Pin, Timer, SPI, ExtInt, delay
    from Stepper_Driver import StepperDriver
    from Actuator_Driver import Actuator
    from Joystick_Driver import Joystick
    
    # HPGL file data
    filename = input('Please enter the name of your drawing file on the MCU. Include .hpgl file extension.')
    dpi = 1016
    offset = 0.1
    
## Program Constants
    #Linear actuation, r [in], per rotation
    N = task_share.Share('f',thread_protect = False, name = "Screw Conversion")
    N.put(0.19685/(2*math.pi))
    theta_guess = np.array([[1],[1]])       #Initial guess for Newton-Raphson

## Class Instantiation
    ## @brief   button_int is an external interrupt button callback pyb.Pin object.
    # @details  Instantiates a pyb.Pin object enabling button interrupt request 
    #           required by the button_callback() method.
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, button_callback)
    
    ## @brief   task_share.Share object used to indicate button interrupt status.
    # @details  button_flag is a shared flag variable used to communicate the status
    #           of the button input on the MCU between tasks in the cotask.py multitasking
    #           program.
    button_flag = task_share.Share('b', thread_protect = False, name = "Button Flag")
    button_flag.put(True)
    
    timer = Timer(4, period=3, prescaler=0)
    ## @brief   pyb.Timer clock object used by TMC4210.
    # @details  CLK is a timer object used by the TMC4210 to calculate acceleration 
    #           ramping functions for control of a stepper motor.
    CLK = timer.channel(1, pin=Pin.cpu.B6, mode=Timer.PWM, pulse_width=2)
    
    ## @brief   pyb.spi serial communication bus object.
    # @details  spi is a SPI bus object used for serial communication between the 
    #           microcontroller and the TMC4210 stepper motor driver.
    spi = SPI(2, SPI.CONTROLLER, baudrate = 1_000_000, polarity = 1, phase = 1)
    
    ## @brief   Linear actuator (ballscrew) stepper motor driver object.
    # @details  DRV1 is the Stepper_Driver.StepperDriver instance which enables 
    #           interaction with the TMC4210 controlling the stepper motor coupled 
    #           to the radial/linear actuator in a polar plotting machine.
    DRV1 = StepperDriver(Pin.cpu.B0,Pin.cpu.C3,spi,CLK)             #Radial Motion
    ## @brief   Turntable actuator stepper motor driver object.
    # @details  DRV2 is the Stepper_Driver.StepperDriver instance which enables 
    #           interaction with the TMC4210 controlling the stepper motor which 
    #           direct drives the turntable drawing surface in a polar plotting machine.
    DRV2 = StepperDriver(Pin.cpu.C0,Pin.cpu.C2,spi,CLK)             #Angular Motion
    ## @brief   Linear actuator motor driver object.
    # @details  Pen is the Actuator_Driver.Actuator instance which enables control 
    #           of a 12V DC linear actuator motor.
    Pen = Actuator()                                                #Linear Actuator
    ## @brief   Analog joystick driver object.
    # @details  Joystick is a Joystick_Driver.Joystick instance which enables interaction 
    #           with a simple analog joystick potentiometer and button layout.
    Joystick = Joystick()
    
## Open .hpgl and pre-compute interpolated xy dataset for storage in text file
    with open(filename, 'r') as f:
        string = f.read()
    data_list = parse_hpgl(string)
    interpolate(data_list)
    
## Multitasking
    # Define multitasking data sharing objects
    ## @brief   task_share.Queue object for sharing pen up and pen down .hpgl commands between tasks.
    # @details  cmd_queue is a task_share.Queue object which allows ordered sharing of the pen 
    #           actuation commands between the generate_thetas() task and the actuate_motors() task.
    cmd_queue = task_share.Queue('f', 25, thread_protect = False, overwrite = False, name = "Motor Commands")
    ## @brief   task_share.Queue object for sharing linear actuator angle .hpgl coordinates between tasks.
    # @details  theta_r is a task_share.Queue object which allows ordered sharing of radial coordinate control 
    #           angles generated by the generate_thetas() task with the actuate_motors() task.
    theta_r = task_share.Queue('f', 25, thread_protect = False, overwrite = False, name = "Ballscrew Thetas")
    ## @brief   task_share.Queue object for sharing turntable actuator angle .hpgl coordinates between tasks.
    # @details  theta_t is a task_share.Queue object which allows ordered sharing of turntable coordinate control 
    #           angles generated by the generate_thetas() task with the actuate_motors() task.
    theta_t = task_share.Queue('f', 25, thread_protect = False, overwrite = False, name = "Turntable Thetas")
    
    # Define tasks for the task manager
    theta_task = cotask.Task(generate_thetas, name = 'Generate Thetas task', priority = 2, period = 70, profile = False, trace = False)
    actuator_task = cotask.Task(actuate_motors, name = 'Actuator task', priority = 1, period = 70, profile = False, trace = False)
    # Append these tasks to the task list
    cotask.task_list.append(theta_task)
    cotask.task_list.append(actuator_task)
    
    
    while True:
        cotask.task_list.pri_sched()
    #print(str(cotask.task_list))
