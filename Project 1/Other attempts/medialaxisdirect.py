import cv2
import numpy as np
import sobeledge
import os

my_path = os.path.abspath(os.path.dirname(__file__))
filenumber = str(input("Which video to open? (1, 2, or 3):\n"))

path = os.path.join(my_path, "..", "Videos",filenumber+".mp4")
vid = cv2.VideoCapture(path)

# Check if video opened successfully
if not vid.isOpened():
    print("Error opening video:", path)
    exit()

# Create MOG2 background subtractor for motion detection
subtractor = cv2.createBackgroundSubtractorMOG2()

# Function for Sobel edge detection
# def sobel_edge_detection(img, ksize=3, threshold=127):
#     # Sobel filters for horizontal and vertical gradients
#     sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
#     sobel_y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])

#     # Calculate image gradients
#     gx = cv2.filter2D(img, cv2.CV_64F, sobel_x)
#     gy = cv2.filter2D(img, cv2.CV_64F, sobel_y)

#     # Calculate gradient magnitude and direction (optional)
#     magnitude = np.sqrt(gx**2 + gy**2)
#     direction = np.arctan2(gy, gx)

#     # Apply thresholding for edge detection
#     edge_map = np.where(np.abs(gx) + np.abs(gy) > threshold, 255, 0)  # Convert to binary image
#     edge_map = edge_map.astype(np.uint8)

#     return edge_map


while True:
    # Read a frame from the video
    ret, img = vid.read()

    # Check if frame is read successfully
    if not ret:
        print("Error reading frame from video:")
        break

    # Resize the frame (optional)
    resized_img = cv2.resize(img, None, fx=0.5, fy=0.5)  # Resize to half

    # Apply background subtraction for motion detection
    foreground_mask = subtractor.apply(resized_img)

    # Apply erosion to remove noise (optional)
    kernel = np.ones((2,2), np.uint8)
    eroded_mask = cv2.erode(foreground_mask, kernel, iterations=3)

    # Apply Sobel edge detection to the eroded mask
    edges = sobeledge.sobel_edge_detection(eroded_mask)

    # Perform Hough line detection on the edges
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=100, minLineLength=50, maxLineGap=10)

    # Create a black image for visualization
    new_img = np.zeros((resized_img.shape[0], resized_img.shape[1], 3), dtype=np.uint8)

    # Draw detected lines on the black image
    # if lines is not None:
    #     for line in lines:
    #         x1, y1, x2, y2 = line[0]
    #         cv2.line(new_img, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw lines in green
    #         # Draw detected lines on the original image

    if lines is not None:
        x1_mean=0
        x2_mean=0
        y1_mean=0
        y2_mean=0
        count = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            x1_mean+=x1
            x2_mean+=x2
            y1_mean+=y1
            y2_mean+=y2
            count+=1
            # cv2.line(resized_img, (x1, y1), (x2, y2), (0,255,0), 2)  # Draw lines in green
        x1_mean=x1_mean//count
        x2_mean=x2_mean//count
        y1_mean=y1_mean//count
        y2_mean=y2_mean//count
        cv2.line(resized_img, (x1_mean, y1_mean), (x2_mean, y2_mean), (0,0,255), 2)  # Draw medial axis in red


    # Display the frame with Hough lines
    cv2.imshow("video1",resized_img)

    # Exit the loop by pressing 'q' key
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# Release the video capture object and destroy all windows
vid.release()
cv2.destroyAllWindows()