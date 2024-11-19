import motor_control
import Robot
import cv
import pid_control
import time
import threading
import numpy as np

K_PID = [0.015, 0.0001, 0.0051]
k = 1
a = 1

x = -1
y = -1
area = -1
goal = [0, 0]

# # motor test
# try:
#     if not ctrl.setup():
#         print("Motor setup failed. Exiting...")
#         quit()

#     for i in range(10):
#         ctrl.move_motor(motor_control.angle_to_position(301.46), 
#                         motor_control.angle_to_position(307.4), 
#                         motor_control.angle_to_position(44.46))
#         time.sleep(3)
#         ctrl.move_motor(motor_control.angle_to_position(321.46), 
#                         motor_control.angle_to_position(327.4), 
#                         motor_control.angle_to_position(64.46))
#         time.sleep(3)


# except Exception as e:
#     print(f"An error occurred: {e}")

# finally:
#     ctrl.shutdown()
#     print("Shutdown complete.")

try:
    robot = Robot.Robot()
    camera = cv.Camera()
    pid = pid_control.PID(K_PID, k, a)

    robot.set_to_initial_position()

    while True:
        # Capture and process each frame
        frame = camera.capture_image()
        if frame is None:
            break  # Stop if frame capture fails

        # Locate ball and display the frame
        x, y, area = camera.locate_ball(frame)
        # camera.display_video(frame)

        # Optional: print the ball's coordinates and area
        if area > 0:
            print(f"Ball located at (x: {x}, y: {y}), Area: {area}")

        Current_value = [x, y, area]

        if x != -1:
            theta, phi = pid.calc(goal, Current_value)

        new_position = [0, 0, robot.starting_position[2]]
        robot.adjust_posture(new_position, 0.01)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    robot.deactivate()
