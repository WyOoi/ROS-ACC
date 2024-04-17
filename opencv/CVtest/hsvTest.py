import cv2
import numpy as np

# Raspberry Pi camera settings
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Set the width of the frame
cap.set(4, 480)  # Set the height of the frame

def nothing(x):
    pass

# Create a window for trackbars
cv2.namedWindow("Trackbars")

# Create trackbars for Hue, Saturation, and Value
cv2.createTrackbar("Hue Min", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("Hue Max", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("Sat Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Sat Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("Val Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Val Max", "Trackbars", 255, 255, nothing)

while True:
    # Capture the video frame
    ret, frame = cap.read()

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get the current values of the trackbars
    hue_min = cv2.getTrackbarPos("Hue Min", "Trackbars")
    hue_max = cv2.getTrackbarPos("Hue Max", "Trackbars")
    sat_min = cv2.getTrackbarPos("Sat Min", "Trackbars")
    sat_max = cv2.getTrackbarPos("Sat Max", "Trackbars")
    val_min = cv2.getTrackbarPos("Val Min", "Trackbars")
    val_max = cv2.getTrackbarPos("Val Max", "Trackbars")

    # Create a mask based on the HSV range
    mask = cv2.inRange(hsv, (hue_min, sat_min, val_min), (hue_max, sat_max, val_max))

    # Apply the mask to the original frame
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Display the original frame, mask, and result
    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    # Check for the 'q' key to quit the program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
