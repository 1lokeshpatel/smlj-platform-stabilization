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
        
    # ctrl.disable_torque(1)
    # ctrl.disable_torque(2)
    # ctrl.disable_torque(3)

    # while 1:
    #     ctrl.read_status()
    # ctrl.groupwrite_num.clearParam()
    # ctrl.set_extended_operating_mode(1)
    # ctrl.set_extended_operating_mode(2)
    # ctrl.set_extended_operating_mode(3)
    ctrl.set_position_limits(1, motor_control.angle_to_position(290), motor_control.angle_to_position(355))
    ctrl.set_position_limits(2, motor_control.angle_to_position(300), motor_control.angle_to_position(355))
    ctrl.set_position_limits(3, motor_control.angle_to_position(40), motor_control.angle_to_position(130))

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
