import motor_control
import Robot
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
        ctrl.move_motor(motor_control.angle_to_position(301.46), 
                        motor_control.angle_to_position(307.4), 
                        motor_control.angle_to_position(44.46))
        time.sleep(3)
        ctrl.move_motor(motor_control.angle_to_position(321.46), 
                        motor_control.angle_to_position(327.4), 
                        motor_control.angle_to_position(64.46))
        time.sleep(3)


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
