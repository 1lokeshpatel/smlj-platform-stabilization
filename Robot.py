# Robot Base Class

import math
import time
import motor_control
import ik2

class Robot:
    # Initialize robot structure
    def __init__(self):
        # Initialize motor control object
        self.ctrl = motor_control.MotorControl()

        # Prepare motors
        self.activate()

        # Define link lengths (base, lower link, upper link, ceiling)
        self.links = [0.05984, 0.1, 0.097, 0.105]

        # Initial posture (theta, phi, pz)
        self.starting_position = [0, 0, 0.015]
        self.max_pz = 0.160
        self.min_pz = 0.0
        self.max_phi = 40
        self.wait_time = 3 #seconds

    # Method to prepare the robot
    def activate(self):
        if not self.ctrl.setup():
            print("Motor setup failed. Exiting...")
            quit()
        self.ctrl.calibrate()

    # Method to shut down the robot
    def deactivate(self):
        self.ctrl.shutdown()
        print("Shutdown complete.")

    # Method to achieve posture (theta, phi, Pz)
    def adjust_posture(self, position, t):
        theta = position[0]
        phi = position[1]
        if phi > self.max_phi:
            phi = self.max_phi
        Pz = position[2]
        Pz = max(self.min_pz, min(Pz, self.max_pz))

        #print(f"Pz: {Pz}")

        motor_angles = ik2.calculate_motor_angles(theta, phi, Pz)
        #print(motor_angles)

        self.ctrl.move_motor(motor_control.angle_to_position(motor_angles[0]),
                                 motor_control.angle_to_position(motor_angles[1]),
                                 motor_control.angle_to_position(motor_angles[2]))

        time.sleep(0)
    
    def set_to_initial_position(self):
        self.adjust_posture(self.starting_position, self.wait_time)
