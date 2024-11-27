import motor_control
import Robot
import cv
import cv2
import pid_control
import time
import threading
import numpy as np

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
waypoints = [(0, 0), (-30, 0), (30, 0), (0, 0), (-30, 0), (0, 0)]  # Line path
goal_index = 0  # Start at the first corner
goal = list(waypoints[goal_index])  # Initialize goal to the first waypoint
tolerance_factor = 0.8  # Factor to adjust tolerance based on ball's area
hold_time = 2  # Time in seconds to hold the ball at a waypoint
holding = False  # Flag to indicate if the robot is holding at a waypoint
hold_start_time = None  # Timestamp when holding began

# Initialize robot and camera
robot = Robot.Robot()
camera = cv.Camera()
pid = pid_control.PID(K_PID, k, a)

frame = None

def get_cam_feed():
    global frame, x, y, area, center_pixel_coords, center, goal
    iterations = 0
    while True:
        # Capture and process each frame
        frame = camera.capture_image()
        if frame is None:
            break  # Stop if frame capture fails

        if iterations < 30:
            center_pixel_coords[0], center_pixel_coords[1] = camera.detect_white_dot(frame)

        if iterations == 30:
            # Calculate the center coordinates and update the goal
            center[0] = center_pixel_coords[0] - camera.frame_height / 2
            center[1] = center_pixel_coords[1] - camera.frame_width / 2
            center[0], center[1] = -center[1], center[0]
            center[0] = int(center[0])
            center[1] = int(center[1])
            # Adjust all waypoints based on the detected center
            waypoints[:] = [[wp[0] + center[0], wp[1] + center[1]] for wp in waypoints]
            goal = list(waypoints[goal_index])  # Reset goal to adjusted first waypoint

        # Draw a red dot at the detected white spot
        cv2.circle(frame, (center_pixel_coords[0], center_pixel_coords[1]), 5, (0, 0, 255), -1)

        x, y, area = camera.locate_ball(frame, center_pixel_coords)

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

        if x != -1 and area > 0:
            # Calculate tolerance in terms of ball's radius
            tolerance = int(tolerance_factor * np.sqrt(area / np.pi))  # Approximate radius

            # Check if the ball is within the tolerance of the current goal
            if abs(x - goal[0]) <= tolerance and abs(y - goal[1]) <= tolerance:
                if not holding:
                    # Start holding at the waypoint
                    hold_start_time = time.time()
                    holding = True
                    print(f"Holding at waypoint: {goal}")
                elif time.time() - hold_start_time >= hold_time:
                    # Move to the next waypoint after holding
                    holding = False
                    goal_index += 1

                    # Stop if the ball reaches the last waypoint
                    if goal_index >= len(waypoints):
                        print("Ball has reached the last waypoint. Stopping.")
                        break

                    # Update the goal to the next waypoint
                    goal = list(waypoints[goal_index])
                    print(f"Moving to next waypoint: {goal}")
            else:
                # Reset holding if the ball moves outside the tolerance
                holding = False

            # Calculate the next posture adjustment
            theta, phi = pid.calc(goal, Current_value)

            new_position = [theta, -phi, 0.015]
            robot.adjust_posture(new_position, 0.01)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    robot.deactivate()
    camera.shutdown_camera()
