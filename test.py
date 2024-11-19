import motor_control
import Robot
import time
import threading
import numpy as np

ctrl = motor_control.MotorControl()

# motor test
try:
    # if not ctrl.setup():
    #     print("Motor setup failed. Exiting...")
    #     quit()

    while(1):
        ctrl.read_status()


except Exception as e:
    print(f"An error occurred: {e}")

finally:
    ctrl.shutdown()
    print("Shutdown complete.")

# try:
#     robot = Robot.Robot()
#     robot.set_to_initial_position()
#     theta = 0
#     phi = 0
#     pz = 0.09
#     new_position = [theta, phi, pz]
#     robot.adjust_posture(new_position, 10)

# except Exception as e:
#     print(f"An error occurred: {e}")

# finally:
#     robot.deactivate()
