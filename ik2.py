import numpy as np

deg_to_rad = np.pi / 180.0
rad_to_deg = 180.0 / np.pi

def calculate_motor_angles(theta, phi, target_height):
    # Define constants in the mechanical design
    theta_rad = theta * deg_to_rad
    phi_rad = phi * deg_to_rad
    lower_shaft_length = 100 # millimeters
    upper_shaft_length = 97.5
    motor_angle_offset = 0.0 # degrees
    # target_height = 100
    target_height *= 1000
    initial_height = 82.05

    # Target position vector (base frame)
    target_position = np.array([0.0, 0.0, target_height])

    platform_radius = 150.0 # millimeters
    platform_vector_motor_1 = np.array([platform_radius, 0.0, initial_height])
    platform_vector_motor_2 = np.array([-platform_radius / 2.0, platform_radius * np.cos(30 * deg_to_rad), initial_height])
    platform_vector_motor_3 = np.array([-platform_radius / 2.0, -platform_radius * np.cos(30 * deg_to_rad), initial_height])
    platform_points = np.array([platform_vector_motor_1, platform_vector_motor_2, platform_vector_motor_3])

    base_radius = 101.35 # millimeters
    base_vector_motor_1 = np.array([base_radius, 0.0, 0.0])
    base_vector_motor_2 = np.array([-base_radius * np.sin(30 * deg_to_rad), base_radius * np.cos(30 * deg_to_rad), 0.0])
    base_vector_motor_3 = np.array([-base_radius * np.sin(30 * deg_to_rad), -base_radius * np.cos(30 * deg_to_rad), 0.0])
    base_points = np.array([base_vector_motor_1, base_vector_motor_2, base_vector_motor_3])

    # k_platform = 300.0
    # k_base = 202.7
    # b1 = np.array([k_base, 0.0, 0.0])
    # b2 = np.array([-k_base * np.sin(30 * deg_to_rad), k_base * np.cos(30 * deg_to_rad), 0.0])
    # b3 = np.array([-k_base * np.sin(30 * deg_to_rad), -k_base * np.cos(30 * deg_to_rad), 0.0])

    # p1 = np.array([k_platform, 0.0, initial_height])
    # p2 = np.array([-k_platform * np.sin(30 * deg_to_rad), k_platform * np.cos(30 * deg_to_rad), initial_height])
    # p3 = np.array([-k_platform * np.sin(30 * deg_to_rad), -k_platform * np.cos(30 * deg_to_rad), initial_height])
    # base_points = np.array([b1, b2, b3])
    # platform_points = np.array([p1, p2, p3])

    # Combine rotation matrices
    # R_x = np.array([
    #     [np.cos(theta), 0, np.sin(theta)],
    #     [0, 1, 0],
    #     [-np.sin(theta), 0, np.cos(theta)]
    # ])
    # R_y = np.array([
    #     [1, 0, 0],
    #     [0, np.cos(phi), -np.sin(phi)],
    #     [0, np.sin(phi), np.cos(phi)]
    # ])
    # R = R_x @ R_y

    rotation_matrix = np.array([
        [np.cos(theta_rad), np.sin(theta_rad) * np.sin(phi_rad), np.cos(phi_rad) * np.sin(theta_rad)],
        [0, np.cos(phi_rad), -np.sin(phi_rad)],
        [-np.sin(theta_rad), np.cos(theta_rad) * np.sin(phi_rad), np.cos(phi_rad) * np.cos(theta_rad)]
    ])

    # Apply transformation: Transform base points to platform frame
    transformed_points = np.dot(rotation_matrix, platform_points.T).T + target_position

    # Calculate vector differences (l1, l2, l3)
    # l_vectors = transformed_points - platform_points # don't think this is right
    l_vectors = transformed_points - base_points


    # Calculate angles using the lengths of the vectors
    def calculate_angle(length):
        # Limit length to prevent overactuation
        # length = np.min([np.max([length, some_max_value]), some_min_value])
        intermed = ((length**2) + (lower_shaft_length**2) - (upper_shaft_length**2)) / (2 * lower_shaft_length * length)
        intermed = np.clip(intermed, -1.0, 1.0) # Ensure within domain for acos
        angle = -rad_to_deg * np.arccos(intermed) + 90.0 + motor_angle_offset
        return angle

    # Compute the motor angles
    motor_angles = np.array([calculate_angle(np.linalg.norm(l)) for l in l_vectors])

    return motor_angles

# print(calculate_motor_angles(0,0, 0.09))
