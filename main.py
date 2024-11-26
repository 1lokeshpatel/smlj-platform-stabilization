import motor_control
import Robot
import cv
import pid_control
import time
import threading
import numpy as np


# best so far is Kp = 0.07, Ki = 0, Kd = 0.01
# another is Kp = 0.03, Ki = 0, Kd = 0.01
# PID parameters
K_PID = [0.07, 0.01, 0.04]
k = 1
a = 1

# Initialize global variables
x = -1
y = -1
area = -1
goal = [0, 0]  # Goal position in image coordinates

robot = Robot.Robot()
camera = cv.Camera()
pid = pid_control.PID(K_PID, k, a)

frame = None
x = None
y = None
area = None


def get_cam_feed():
    global frame, x, y, area
    while True:
        # Capture and process each frame
        frame = camera.capture_image()
        if frame is None:
            break  # Stop if frame capture fails

        x, y, area = camera.locate_ball(frame, goal)

        # Optional: print the ball's coordinates and area
        # if area > 0:
        #     print(f"Ball located at (x: {x}, y: {y}), Area: {area}")
        #     print("Finding ball")

        # Display the updated frame
        camera.display_video(frame)

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
