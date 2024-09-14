from motor_wrapper import Can_Wrapper
import numpy as np
from six_dof_pid import PID

class PIDInterface:
    def __init__(self, shared_memory_object):
        self.shared_memory_object = shared_memory_object
        self.motor_wrapper = Can_Wrapper()
        self.motor_array = self.motor_wrapper.motors
        # array of PID k Values
        # rows = x, y, z, roll, pitch, yaw
        # columns = kp, ki, kd
        self.K_array = np.array([1, 1, 1, 1, 1, 1], 
                                [1, 1, 1, 1, 1, 1],
                                [1, 1, 1, 1, 1, 1])
        
    def run_loop():
        while self.shared_memory_object.running.value:
            # get the current state of the robot
            current_state = self.shared_memory_object.current_state
            # get the desired state of the robot
            desired_state = self.shared_memory_object.desired_state
            # calculate the error between the current and desired states
            error = desired_state - current_state
            # calculate the PID values for each motor
            pid_values = self.calculate_pid_values(error)
            # send the PID values to the motors
            self.send_pid_values(pid_values)


    def calculate_pid_values(self, error):
        pid_values = np.zeros(6)
        for i in range(6):
            pid_values[i] = self.K_array[i][0] * error[i] + self.K_array[i][1] * self.integral[i] + self.K_array[i][2] * (error[i] - self.prev_error[i])
            self.integral[i] += error[i]
            self.prev_error[i] = error[i]
        return pid_values
    
    def send_pid_values(self, pid_values):
        for i in range(6):
            self.motor_wrapper.move_forward(pid_values[i])

            


        
        
