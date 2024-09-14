import can
import math
import numpy as np
import time

'''
    discord: @kialli
    github: @kchan5071
    
    This class is a wrapper for the CAN bus interface. It is used to send commands to the motors.
    contains:
        move methods for motors
        twos compliment conversion

        test sequence if called as main
'''


class Can_Wrapper:

    def __init__(self):
        #set CAN device
        self.bus = can.Bus(interface='socketcan',channel = 'can0', receive_own_messages=True)

        self.MAX_MOTOR_VAL = 100
    
        #set ~10 for in air, ~30 in water---------------------------------------------------------------
        self.REASONABLE_MOTOR_MAX = 10
        #-------------------------------------------------------------------------------------------------

        self.motors = [
            #LjoyX   LjoyY   RjoyX   RjoyY    Rtrig   Ltrig   LPad       RDpad
            
           #
            [ 0,      -1,      -1,       0,      -1,      1], # motor 0 (top front left)
            [ 1,       0,       0,      -1,       0,      0], # motor 1 (bottom front left)
            [ 0,       1,      -1,       0,      -1,      1], # motor 2 (top back left)
            [ 1,       0,       0,      -1,       0,      0], # motor 3 (bottom back left)
            [ 0,       1,       1,       0,      -1,      1], # motor 4 (top back right)
            [-1,       0,       0,      -1,       0,      0], # motor 5 (bottom back right)
            [ 0,      -1,       1,       0,      -1,      1], # motor 6 (top front right)
            [-1,       0,       0,      -1,       0,      0]  # motor 7 (bottom front right)
        ]
        self.input_list = [0, 0, 0, 0, 0, 0, 0, 0]

    #custom clamp for REASONABLE MOTOR MAX
    #returns number type
    def clamp(self, num):
        ret = max(-1 * self.REASONABLE_MOTOR_MAX, num)
        ret = min(self.REASONABLE_MOTOR_MAX, num)
        return ret

    #custom two's compliment for 2 byte values
    #returns int
    def twos_complement(self, value):
        if (value < 0):
            value = 255 - abs(value)
        return value

    def move_forward(self, value):
        self.input_list[1] = self.clamp(self.input_list[1] + value)

    def move_backward(self, value):
        self.input_list[1] = self.clamp(self.input_list[1] + -value)

    def move_left(self, value):
        self.input_list[0] = self.clamp(self.input_list[0] + value)

    def move_right(self, value):
        self.input_list[0] = self.clamp(self.input_list[0] + -value)

    def move_up(self, value):
        self.input_list[4] = self.clamp(self.input_list[4] + value)

    def move_down(self, value):
        self.input_list[5] = self.clamp(self.input_list[5] + value)

    def turn_up(self, value):
        self.input_list[3] = self.clamp(self.input_list[3] + value)

    def turn_down(self, value):
        self.input_list[4] = self.clamp(self.input_list[4] + -value)

    def turn_left(self, value):
        self.input_list[2] = self.clamp(self.input_list[2] + value)

    def turn_right(self, value):
        self.input_list[2] = self.clamp(self.input_list[2] + -value)


    def stop(self):
        self.input_list = [0,0,0,0,0,0,0,0]


    #sends commands to motors
    def send_command(self):
        thrust_list = []
        for motor in self.motors:
            thrust_list.append(int(self.REASONABLE_MOTOR_MAX * np.dot(motor, self.input_list)))
        #initialize motor value and command string
        motor_value = 0
        command = ""
        for motor_value_from_list in thrust_list:
            motor_value = self.twos_complement(int(motor_value_from_list))
            #format command in HEX, getting rid of the first two characters
            command += '{:02X}'.format(motor_value) + " "

        #init CAN command message
        try:
            message = can.Message(arbitration_id = 16, is_extended_id = False, data = bytearray.fromhex(command))
            self.bus.send(message, timeout = 0.2)
        except:
            print(message)
            print(thrust_list)