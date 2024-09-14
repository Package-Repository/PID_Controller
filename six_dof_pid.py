import numpy as np

class PID:
    def __init__(self, kp, ki, kd, dt, motor_array):
        self.motor_array = motor_array
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.prev_error = [0, 0, 0, 0, 0, 0]
        self.integral = [0, 0, 0, 0, 0, 0]

    def update(self, initial_state, desired_state):
        
    


