import cv
import motor_control
import time
import threading
import numpy as np

# motor test
try:
    ctrl = motor_control.MotorControl()
    if not ctrl.setup():
        print("Motor setup failed. Exiting...")
        quit()

    ctrl.move_motor(motor_control.angle_to_position(25), 
                    motor_control.angle_to_position(25), 
                    motor_control.angle_to_position(25))

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    ctrl.shutdown()
    print("Shutdown complete.")
