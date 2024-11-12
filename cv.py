# Computer Vision Homing Algorithm

import cv2 as cv
import numpy as np
import threading

# Lock for thread safety
lock = threading.Lock()

class Camera:
    def __init__(self):
        # Initialize video capture
        self.cap = cv.VideoCapture(0)
        self.frame_rate = 30  # Set frame rate to 30 fps
        self.cap.set(cv.CAP_PROP_FPS, self.frame_rate)

        # Set resolution for captured frames
        self.frame_height = 480
        self.frame_width = 480

        # Define HSV color range for yellowish-orange
        self.ball_lower_bound = np.array([15, 100, 100])  # H: around 15 degrees
        self.ball_upper_bound = np.array([35, 255, 255])  # H: up to 35 degrees

    def capture_image(self):
        # Capture frame from the camera
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to grab frame.")
            return None
        # Resize frame to desired resolution
        frame_resized = cv.resize(frame, (self.frame_width, self.frame_height))
        return frame_resized

    def display_video(self, image):
        cv.imshow("Live Feed", image)
        cv.waitKey(1)

    def locate_ball(self, image):
        # Convert to HSV color space
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        # Generate mask based on the yellowish-orange color range
        mask = cv.inRange(hsv_image, self.ball_lower_bound, self.ball_upper_bound)
        # Find contours in the mask
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            # Identify the largest contour
            largest_contour = max(contours, key=cv.contourArea)
            # Get the minimum enclosing circle for the contour
            (x, y), radius = cv.minEnclosingCircle(largest_contour)
            area = cv.contourArea(largest_contour)  # Calculate contour area
            if area > 200:  # Ignore noise based on area threshold
                # Draw the circle on the image
                cv.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                # Adjust coordinates to center of image
                x -= self.frame_height / 2
                y -= self.frame_width / 2
                x, y = -y, x
                return int(x), int(y), int(area)  # Return adjusted coordinates and area
        return -1, -1, 0  # Return if no ball is detected

    def shutdown_camera(self):
        # Release camera and close windows
        self.cap.release()
        cv.destroyAllWindows()

# Instantiate and run the camera
camera = Camera()

try:
    while True:
        # Capture and process each frame
        frame = camera.capture_image()
        if frame is None:
            break  # Stop if frame capture fails

        # Locate ball and display the frame
        x, y, area = camera.locate_ball(frame)
        camera.display_video(frame)

        # Optional: print the ball's coordinates and area
        if area > 0:
            print(f"Ball located at (x: {x}, y: {y}), Area: {area}")

        # Exit on pressing 'q'
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Ensure resources are released
    camera.shutdown_camera()