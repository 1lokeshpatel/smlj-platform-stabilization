import motor_control
import Robot
import cv
import pid_control
import time
import threading
import numpy as np

# PID parameters
K_PID = [0.07, 0, 0.01]
k = 1
a = 1

# Initialize global variables
x = -1
y = -1
area = -1
waypoints = [(-120, -120), (-120, 120), (120, 120), (120, -120)]  # Square corners
goal_index = 0  # Start at the first corner
goal = list(waypoints[goal_index])  # Initialize goal to the first waypoint
tolerance_factor = 0.8  # Factor to adjust tolerance based on ball's area

# Initialize robot and camera
robot = Robot.Robot()
camera = cv.Camera()
pid = pid_control.PID(K_PID, k, a)

frame = None

def get_cam_feed():
    global frame, x, y, area
    while True:
        frame = camera.capture_image()
        if frame is None:
            break  # Stop if frame capture fails

        x, y, area = camera.locate_ball(frame)

        camera.display_video(frame)
        time.sleep(0)

try:
    robot.set_to_initial_position()

    cam_thread = threading.Thread(target=get_cam_feed)
    cam_thread.start()

    while True:
        Current_value = [x, y, area]

        if x != -1 and area > 0:
            # Calculate tolerance in terms of ball's radius
            tolerance = int(tolerance_factor * np.sqrt(area / np.pi))  # Approximate radius

            # Check if the ball is within the tolerance of the current goal
            if abs(x - goal[0]) <= tolerance and abs(y - goal[1]) <= tolerance:
                # Move to the next corner
                goal_index += 1

                # Stop if the ball reaches the last corner
                if goal_index >= len(waypoints):
                    print("Ball has reached the last corner. Stopping.")
                    break

                # Update the goal to the next corner
                goal = list(waypoints[goal_index])

            theta, phi = pid.calc(goal, Current_value)

            new_position = [theta, -phi, 0.015]
            robot.adjust_posture(new_position, 0.01)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    robot.deactivate()
    camera.shutdown_camera()
