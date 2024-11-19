import math
import time

class PID:
    def __init__(self, PID, k, alpha):
        self.kp = PID[0]
        self.ki = PID[1]
        self.kd = PID[2]
        self.k = k
        self.alpha = alpha
        self.prev_x = 0
        self.prev_y = 0
        self.prev_err_x = 0
        self.prev_err_y = 0
        self.prev_time = 0
        self.integral_x = 0
        self.integral_y = 0
        self.count = 0
    
    def calc(self, curr_val, goal):
        curr_time = time.perf_counter()
        if self.prev_time == 0:
            self.prev_time = curr_time
            return 0, 0
        
        curr_err_x = goal[0] - curr_val[0]
        curr_err_y = goal[1] - curr_val[1]
        time_diff = curr_time - self.prev_time
        self.integral_x += curr_err_x * time_diff
        self.integral_y += curr_err_y * time_diff

        derivative_x = (curr_err_x - self.prev_err_x) / time_diff
        derivative_y = (curr_err_x - self.prev_err_x) / time_diff

        x = self.kp * curr_err_x + self.ki * self.integral_x + self.kd * derivative_x
        y = self.kp * curr_err_y + self.ki * self.integral_y + self.kd * derivative_y

        x = self.alpha * x + (1-self.alpha) * self.prev_x
        y = self.alpha * y + (1-self.alpha) * self.prev_y

        theta = math.degrees(math.atan2(y,x))

        if theta < 0:
            theta += 360

        phi = self.k * math.sqrt(x**2 + y**2)

        self.prev_err_x = curr_err_x
        self.prev_err_y = curr_err_y
        self.prev_x = x
        self.prev_y = y
        self.prev_time = curr_time
        return theta, phi