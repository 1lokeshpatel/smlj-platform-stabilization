# Robot Base Class

import math
import time

class Robot:
    # Initialize robot structure
    def __init__(self, servo_ids):  # Takes a list of servo IDs
        # # Set up the serial port
        # self.port = cs.sPort()
        # # Initialize servos
        # self.servo1 = cs.RS304MD(servo_ids[0])
        # self.servo2 = cs.RS304MD(servo_ids[1])
        # self.servo3 = cs.RS304MD(servo_ids[2])

        # Define link lengths (base, lower link, upper link, ceiling)
        self.links = [0.04, 0.04, 0.065, 0.065]
        # Initial posture (theta, phi, pz)
        self.starting_position = [0, 0, 0.0632]
        self.max_pz = 0.0732
        self.min_pz = 0.0532
        self.max_phi = 20

    # Method to prepare the robot
    def activate(self):
        # Turn on servo torque
        self.servo1.trq_set(1)
        self.servo2.trq_set(1)
        self.servo3.trq_set(1)

    # Method to shut down the robot
    def deactivate(self):
        # Turn off servo torque
        self.servo1.trq_set(0)
        self.servo2.trq_set(0)
        self.servo3.trq_set(0)
        # Close the serial port
        self.port.close_sPort()

    # Method to calculate inverse kinematics
    def inverse_kinematics(self, normal_vector, Pz):
        # TO-DO - Update with our inverse kinematics equations
        
        # L = self.links
        # # Calculate height of servo reference point (reverse at Pmz)
        # A = (L[0] + L[1]) / Pz
        # B = (Pz**2 + L[2]**2 - (L[0] + L[1])**2 - L[3]**2) / (2 * Pz)
        # C = A**2 + 1
        # D = 2 * (A * B - (L[0] + L[1]))
        # E = B**2 + (L[0] + L[1])**2 - L[2]**2
        # Pmx = (-D + math.sqrt(D**2 - 4 * C * E)) / (2 * C)
        # Pmz = math.sqrt(L[2]**2 - Pmx**2 + 2 * (L[0] + L[1]) * Pmx - (L[0] + L[1])**2)

        # # Calculate angle for servo a
        # ax = (L[3] / (math.sqrt(normal_vector[0]**2 + normal_vector[2]**2))) * normal_vector[2]
        # ay = 0
        # az = Pz + (L[3] / (math.sqrt(normal_vector[0]**2 + normal_vector[2]**2))) * -normal_vector[0]
        # A_m = [ax, ay, az]
        # A = (L[0] - A_m[0]) / A_m[2]
        # B = (A_m[0]**2 + A_m[1]**2 + A_m[2]**2 - L[2]**2 - L[0]**2 + L[1]**2) / (2 * A_m[2])
        # C = A**2 + 1
        # D = 2 * (A * B - L[0])
        # E = B**2 + L[0]**2 - L[1]**2
        # x_a = (-D + math.sqrt(D**2 - 4 * C * E)) / (2 * C)
        # z_a = math.sqrt(L[1]**2 - x_a**2 + 2 * L[0] * x_a - L[0]**2)
        # if az < Pmz:
        #     z_a = -z_a
        # A2 = [x_a, ay, z_a]
        # theta_a = 90 - math.degrees(math.atan2(A2[0] - L[0], A2[2]))

        # # Calculate angle for servo b
        # bx = (L[3] / (math.sqrt(normal_vector[0]**2 + 3 * normal_vector[1]**2 + 4 * normal_vector[2]**2 + 2 * math.sqrt(3) * normal_vector[0] * normal_vector[1]))) * -normal_vector[2]
        # by = (L[3] / (math.sqrt(normal_vector[0]**2 + 3 * normal_vector[1]**2 + 4 * normal_vector[2]**2 + 2 * math.sqrt(3) * normal_vector[0] * normal_vector[1]))) * -math.sqrt(3) * normal_vector[2]
        # bz = Pz + (L[3] / (math.sqrt(normal_vector[0]**2 + 3 * normal_vector[1]**2 + 4 * normal_vector[2]**2 + 2 * math.sqrt(3) * normal_vector[0] * normal_vector[1]))) * (math.sqrt(3) * normal_vector[1] + normal_vector[0])
        # B_m = [bx, by, bz]

        # A = -(B_m[0] + math.sqrt(3) * B_m[1] + 2 * L[0]) / B_m[2]
        # B = (B_m[0]**2 + B_m[1]**2 + B_m[2]**2 + L[1]**2 - L[2]**2 - L[0]**2) / (2 * B_m[2])
        # C = A**2 + 4
        # D = 2 * A * B + 4 * L[0]
        # E = B**2 + L[0]**2 - L[1]**2
        # x_b = (-D - math.sqrt(D**2 - 4 * C * E)) / (2 * C)
        # z_b = math.sqrt(L[1]**2 - 4 * x_b**2 - 4 * L[0] * x_b - L[0]**2)
        # if bz < Pmz:
        #     z_b = -z_b
        # theta_b = 90 - math.degrees(math.atan2(math.sqrt(x_b**2 + by**2) - L[0], z_b))

        # # Calculate angle for servo c
        # cx = (L[3] / (math.sqrt(normal_vector[0]**2 + 3 * normal_vector[1]**2 + 4 * normal_vector[2]**2 - 2 * math.sqrt(3) * normal_vector[0] * normal_vector[1]))) * -normal_vector[2]
        # cy = (L[3] / (math.sqrt(normal_vector[0]**2 + 3 * normal_vector[1]**2 + 4 * normal_vector[2]**2 - 2 * math.sqrt(3) * normal_vector[0] * normal_vector[1]))) * math.sqrt(3) * normal_vector[2]
        # cz = Pz + (L[3] / (math.sqrt(normal_vector[0]**2 + 3 * normal_vector[1]**2 + 4 * normal_vector[2]**2 - 2 * math.sqrt(3) * normal_vector[0] * normal_vector[1]))) * (-math.sqrt(3) * normal_vector[1] + normal_vector[0])
        # C_m = [cx, cy, cz]

        # A = -(C_m[0] - math.sqrt(3) * C_m[1] + 2 * L[0]) / C_m[2]
        # B = (C_m[0]**2 + C_m[1]**2 + C_m[2]**2 + L[1]**2 - L[2]**2 - L[0]**2) / (2 * C_m[2])
        # C = A**2 + 4
        # D = 2 * A * B + 4 * L[0]
        # E = B**2 + L[0]**2 - L[1]**2
        # x_c = (-D - math.sqrt(D**2 - 4 * C * E)) / (2 * C)
        # z_c = math.sqrt(L[1]**2 - 4 * x_c**2 - 4 * L[0] * x_c - L[0]**2)
        # if cz < Pmz:
        #     z_c = -z_c
        # theta_c = 90 - math.degrees(math.atan2(math.sqrt(x_c**2 + cy**2) - L[0], z_c))

        return [theta_a, theta_b, theta_c]

    # Method to achieve posture (theta, phi, Pz) within time t
    def adjust_posture(self, position, t):
        theta = position[0]
        phi = position[1]
        if phi > self.max_phi:
            phi = self.max_phi
        Pz = position[2]
        Pz = max(self.min_pz, min(Pz, self.max_pz))

        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))
        normal_vector = [x, y, z]

        angles = self.inverse_kinematics(normal_vector, Pz)
        self.servo1.control_time_rotate(angles[0], t)
        self.servo2.control_time_rotate(angles[1], t)
        self.servo3.control_time_rotate(angles[2], t)
        time.sleep(t)
    
    def set_to_initial_position(self):
        self.adjust_posture(self.starting_position, 1)
