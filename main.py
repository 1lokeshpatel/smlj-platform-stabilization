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
        
    ctrl.disable_torque(1)
    ctrl.disable_torque(2)
    ctrl.disable_torque(3)

    while 1:
        ctrl.read_status()
    # for i in range(10):
    #     ctrl.move_motor(motor_control.angle_to_position(185), 
    #                     motor_control.angle_to_position(185), 
    #                     motor_control.angle_to_position(185))
    #     time.sleep(3)
    #     ctrl.move_motor(motor_control.angle_to_position(190), 
    #                     motor_control.angle_to_position(190), 
    #                     motor_control.angle_to_position(190))

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    ctrl.shutdown()
    print("Shutdown complete.")
