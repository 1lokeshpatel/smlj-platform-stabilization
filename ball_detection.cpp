#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Function to detect a yellow ball
void detectYellowBall() {
    // Open the default camera
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cout << "Error: Could not open the camera." << endl;
        return;
    }
    
    // Set the frame rate and resolution
    cap.set(CAP_PROP_FPS, 30);

    while (true) {
        Mat frame;
        // Capture a new frame
        cap >> frame;
        if (frame.empty()) {
            cout << "Failed to grab frame" << endl;
            break;
        }

        // Resize the frame to 480x480
        resize(frame, frame, Size(480, 480));

        // Convert the frame to HSV color space
        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        // Define the range for yellow color in HSV
        Scalar lower_yellow(20, 100, 100); // Lower bound for HSV values
        Scalar upper_yellow(30, 255, 255); // Upper bound for HSV values

        // Threshold the HSV image to get only yellow colors
        Mat mask;
        inRange(hsv, lower_yellow, upper_yellow, mask);

        // Find contours
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        if (!contours.empty()) {
            // Get the largest contour by area
            auto largest_contour = *max_element(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {
                return contourArea(c1) < contourArea(c2);
            });
            // Get the minimum enclosing circle of the largest contour
            Point2f center;
            float radius;
            minEnclosingCircle(largest_contour, center, radius);

            // Only consider large enough objects
            if (radius > 10) {
                // Draw the circle around the detected yellow ball
                circle(frame, center, static_cast<int>(radius), Scalar(0, 255, 255), 2);
                // Draw a red dot at the center
                circle(frame, center, 2, Scalar(0, 0, 255), -1);
                cout << "Yellow ball detected at position: (" << static_cast<int>(center.x) << ", " << static_cast<int>(center.y) << ")" << endl;
            }
        }

        // Display the result
        imshow("Frame", frame);

        // Break the loop if 'q' is pressed
        if (waitKey(1) == 'q') {
            break;
        }
    }

    // Release resources
    cap.release();
    destroyAllWindows();
}

int main() {
    // Call the function to detect the yellow ball
    detectYellowBall();
    return 0;
}