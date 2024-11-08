import math
import time

class PID:
    def __init__(self, PID):
        self.kp = PID[0]
        self.ki = PID[1]
        self.kd = PID[2]
        self.prev_x = 0
        self.prev_y = 0
        self.prev_err_x = 0
        self.prev_err_y = 0
        self.prev_time = 0
    
    def calc(self, curr_val, goal):
        curr_err_x = goal[0] - curr_val[0]
        curr_err_y = goal[1] - curr_val[1]
        theta = 0
        phi = 0
        return theta, phi
    # TODO: make PID class, very similar to online examples probably