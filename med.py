import cv2
import numpy as np

# Define video file path
video_path = "Videos/1.mp4"

# Open the video capture object
vid = cv2.VideoCapture(video_path)

# Check if video opened successfully
if not vid.isOpened():
    print("Error opening video:", video_path)
    exit()

# Create MOG2 background subtractor for motion detection
subtractor = cv2.createBackgroundSubtractorMOG2()

while True:
    # Read a frame from the video
    ret, img = vid.read()

    # Check if frame is read successfully
    if not ret:
        print("Error reading frame from video:")
        break  # Exit the loop if frame reading fails

    # Resize the frame (optional, adjust parameters as needed)
    resized_img = cv2.resize(img, None, fx=0.5, fy=0.5)  # Resizes to half the original size

    # Apply background subtraction for motion detection
    foreground_mask = subtractor.apply(resized_img)

    # Apply erosion to remove noise (optional, adjust parameters as needed)
    kernel = np.ones((3, 3), np.uint8)
    eroded_mask = cv2.erode(foreground_mask, kernel, iterations=2)

    # Detect edges using Canny edge detection
    edges = cv2.Canny(eroded_mask, 50, 150, apertureSize=3)

    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=100, minLineLength=50, maxLineGap=10) 
    # Adjust parameters as needed

    
    # Draw detected lines on the original image
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(resized_img, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw lines in green

        # Display the frame with detected lines
    cv2.imshow("video with Hough lines",resized_img)

    # Exit the loop by pressing 'q' key
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# Release the video capture object and destroy all windows
vid.release()
cv2.destroyAllWindows()