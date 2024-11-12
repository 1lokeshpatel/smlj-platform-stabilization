import motor_control
import time
import threading
import numpy as np

ctrl = motor_control.MotorControl()

# motor test
try:
    if not ctrl.setup():
        print("Motor setup failed. Exiting...")
        quit()

    for i in range(10):
        ctrl.move_motor(motor_control.angle_to_position(210), 
                        motor_control.angle_to_position(210), 
                        motor_control.angle_to_position(210))

        ctrl.move_motor(motor_control.angle_to_position(260), 
                        motor_control.angle_to_position(260), 
                        motor_control.angle_to_position(260))

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    ctrl.shutdown()
    print("Shutdown complete.")
