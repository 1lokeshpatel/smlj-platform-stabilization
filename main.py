import cv
import motor_control
import time
import threading
import numpy as np

# motor test
try:
    ctrl = motor_control.MotorControl()
    ctrl.setup()
    ctrl.enable_torque()
    ctrl.move_motor(motor_control.angle_to_position(25), motor_control.angle_to_position(25), motor_control.angle_to_position(25))
    ctrl.shutdown()

except Exception as e:
        print(f"An error occurred: {e}")

finally:
    # Shutdown and cleanup resources
    motor_control.shutdown()
    print("Shutdown complete.")