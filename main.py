import motor_control
import Robot
import cv
import cv2
import pid_control
import time
import threading
import numpy as np


# Kp = 0.07, Ki = 0, Kd = 0.01
# Kp = 0.03, Ki = 0, Kd = 0.01
# Kp = 0.07, Ki = 0.005, Kd = 0.04
# Kp = 0.07, Ki 0.011, Kd 0.035
# Kp = 0.08, Ki 0.0, Kd 0.04

# PID parameters
K_PID = [0.07, 0.011, 0.035]
k = 1
a = 1

# Initialize global variables
x = -1
y = -1
area = -1
center_pixel_coords = [0, 0]
center = [0, 0]
goal = [0, 0]  # Goal position

robot = Robot.Robot()
camera = cv.Camera()
pid = pid_control.PID(K_PID, k, a)

frame = None
x = None
y = None
area = None


def get_cam_feed():
    global frame, x, y, area
    iterations = 0
    while True:
        # Capture and process each frame
        frame = camera.capture_image()
        if frame is None:
            break  # Stop if frame capture fails

        if iterations < 30:
            center_pixel_coords[0], center_pixel_coords[1] = camera.detect_white_dot(frame)
            iterations += 1
            continue

        if iterations == 30:
            center[0] = center_pixel_coords[0] - camera.frame_height / 2
            center[1] = center_pixel_coords[1] - camera.frame_width / 2
            center[0], center[1] = -center[1], center[0]
            center[0] = int(center[0])
            center[1] = int(center[1])
            goal = [goal[0] + center[0], goal[1] + center[1]]
            print("WORKS")

        # Draw a red dot at the detected white spot
        cv2.circle(frame, (center_pixel_coords[0], center_pixel_coords[1]), 5, (0, 0, 255), -1)

        x, y, area = camera.locate_ball(frame, goal)

        # Optional: print the ball's coordinates and area
        # if area > 0:
        #     print(f"Ball located at (x: {x}, y: {y}), Area: {area}")
        #     print("Finding ball")

        # Display the updated frame
        camera.display_video(frame)

        iterations += 1

        time.sleep(0)


try:
    robot.set_to_initial_position()

    # Start the camera feed thread
    cam_thread = threading.Thread(target=get_cam_feed)
    cam_thread.start()

    while True:
        Current_value = [x, y, area]

        if x != -1:
            theta, phi = pid.calc(goal, Current_value)

        new_position = [theta, -phi, 0.015]
        robot.adjust_posture(new_position, 0.01)


except Exception as e:
    print(f"An error occurred: {e}")


finally:
    robot.deactivate()
    camera.shutdown_camera()
