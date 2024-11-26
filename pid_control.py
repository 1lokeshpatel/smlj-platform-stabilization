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

        # Calculate the current errors in x and y
        curr_err_x = goal[0] - curr_val[0]
        curr_err_y = goal[1] - curr_val[1]

        time_diff = curr_time - self.prev_time
        self.integral_x += curr_err_x * time_diff
        self.integral_y += curr_err_y * time_diff

        # Calculate derivatives for x and y errors
        derivative_x = (curr_err_x - self.prev_err_x) / time_diff
        derivative_y = (curr_err_y - self.prev_err_y) / time_diff

        # PID control for x (roll) and y (pitch)
        roll = self.kp * curr_err_x + self.ki * self.integral_x + self.kd * derivative_x
        pitch = self.kp * curr_err_y + self.ki * self.integral_y + self.kd * derivative_y

        print(f"X Error: {round(curr_err_x, 3):<8}   Kp_X Error: {round(self.kp * curr_err_x, 3):<8}   Ki_X Error: {round(self.ki * self.integral_x, 3):<8}   Kd_X Error: {round(self.kd * derivative_x, 3):<8}   Roll: {round(roll, 3):<8}")
        print(f"Y Error: {round(curr_err_y, 3):<8}   Kp_Y Error: {round(self.kp * curr_err_y, 3):<8}   Ki_Y Error: {round(self.ki * self.integral_y, 3):<8}   Kd_Y Error: {round(self.kd * derivative_y, 3):<8}   Pitch: {round(pitch, 3):<8}")

        # Apply smoothing (low pass filter)
        roll = self.alpha * roll + (1 - self.alpha) * self.prev_x
        pitch = self.alpha * pitch + (1 - self.alpha) * self.prev_y

        # Roll and pitch represent theta and phi respectively
        theta = roll  # Roll angle
        phi = pitch  # Pitch angle

        # Update previous values for the next iteration
        self.prev_err_x = curr_err_x
        self.prev_err_y = curr_err_y
        self.prev_x = roll
        self.prev_y = pitch
        self.prev_time = curr_time

        return theta, phi
