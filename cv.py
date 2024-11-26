# Computer Vision Homing Algorithm

import cv2 as cv
import numpy as np

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

    def detect_white_dot(self, image):
        # Convert to grayscale for detecting white dots
        gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        _, white_mask = cv.threshold(gray_image, 200, 255, cv.THRESH_BINARY)
        white_contours, _ = cv.findContours(white_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if white_contours:
            # Find the contour closest to the image center
            center_x, center_y = self.frame_width // 2, self.frame_height // 2

            # Combine proximity to the center and contour size
            def scoring_function(contour):
                dist_to_center = cv.pointPolygonTest(contour, (center_x, center_y), True)
                size_penalty = -cv.contourArea(contour)
                return dist_to_center + 0.1 * size_penalty

            # Select the best contour based on the scoring function
            best_contour = max(white_contours, key=scoring_function)

            # Compute the center of the best contour
            M = cv.moments(best_contour)
            if M["m00"] != 0:
                white_dot_x = int(M["m10"] / M["m00"])
                white_dot_y = int(M["m01"] / M["m00"])
                return white_dot_x, white_dot_y
        return None, None

    def locate_ball(self, image, goal):
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
                # Draw the enclosing circle on the image (red)
                cv.circle(image, (int(x), int(y)), int(radius), (0, 0, 255), 2)
                # Draw a red circle at the center of the ball
                cv.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)

                # Convert goal to image coordinates
                goal_x, goal_y = goal[1], -goal[0]
                goal_x = int(goal_x + self.frame_height / 2)
                goal_y = int(goal[1] + self.frame_width / 2)
                # goal_x = int(goal[0] + self.frame_width / 2)
                # goal_y = int(goal[1] + self.frame_height / 2)

                # Draw a red line from the center of the ball to the goal position
                cv.line(image, (int(x), int(y)), (goal_x, goal_y), (0, 0, 255), 2)

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
