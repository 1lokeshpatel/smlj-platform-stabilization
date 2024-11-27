from cgitb import enable
import motor_control
import Robot
import cv
import cv2
import pid_control
import time
import threading
import numpy as np

# PID parameters
K_PID = [0.08, 0.03, 0.035]
k = 1
a = 1

# Initialize global variables
x = -1
y = -1
area = -1
center_pixel_coords = [0, 0]
center = [0, 0]
waypoints = [(0, 0), (-40, 0), (-10, 0), (-40, 0), (0, 0), (40, 0), (100, 0), (40, 0), (0, 0)]  # Line path
goal_index = 0  # Start at the first corner
goal = list(waypoints[goal_index])  # Initialize goal to the first waypoint
tolerance_factor = 1.2  # Factor to adjust tolerance based on ball's area
hold_time = 1.5  # Time in seconds to hold the ball at a waypoint
holding = False  # Flag to indicate if the robot is holding at a waypoint
hold_start_time = None  # Timestamp when holding began
enable_d = True

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

        # Draw a red dot at the detected white spot
        cv2.circle(frame, (240, 240), 5, (0, 0, 255), -1)

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
                    print("Moving to next waypoint FLAG")
                    # Move to the next waypoint after holding
                    holding = False
                    goal_index += 1
                    enable_d = False

                    # Stop if the ball reaches the last waypoint
                    if goal_index >= len(waypoints):
                        print("Ball has reached the last waypoint. Stopping.")
                        break

                    # Update the goal to the next waypoint
                    goal = list(waypoints[goal_index])
                    print(f"Moving to next waypoint: {goal}")
            else:
                # Reset holding if the ball moves outside the tolerance
                print("ball is out of position")
                holding = False
                hold_start_time = None
                enable_d = True

            # Calculate the next posture adjustment
            theta, phi = pid.calc(goal, Current_value)
            if enable_d:
                new_position = [theta, -phi, 0.015]
            
            robot.adjust_posture(new_position, 0.01)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    robot.deactivate()
    camera.shutdown_camera()
