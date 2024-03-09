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

# Define Sobel edge detection function outside the loop
def sobel_edge_detection(img, ksize=3, threshold=127):
    # Sobel filters for horizontal and vertical gradients
    sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
    sobel_y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])

    # Calculate image gradients
    gx = cv2.filter2D(img, cv2.CV_64F, sobel_x)
    gy = cv2.filter2D(img, cv2.CV_64F, sobel_y)

    # Calculate gradient magnitude and direction (optional)
    magnitude = np.sqrt(gx**2 + gy**2)
    direction = np.arctan2(gy, gx)

    # Apply thresholding for edge detection
    edge_map = np.where(np.abs(gx) + np.abs(gy) > threshold, 255, 0)  # Convert to binary image

    return edge_map

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

    # Apply Sobel edge detection to the eroded mask
    edges = sobel_edge_detection(eroded_mask)  # Apply to edges for targeted line detection
    

    # Create a black image for Hough lines
    new_img = np.zeros((resized_img.shape[0], resized_img.shape[1], 3), dtype=np.uint8)

    # lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=100, minLineLength=50, maxLineGap=10)

    # # Draw detected lines on the black image
    # if lines is not None:
    #     for line in lines:
    #         x1, y1, x2, y2 = line[0]
    #         cv2.line(new_img, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw lines in green

    # # Display the frame with Hough lines
    cv2.imshow("video with Hough lines",edges)


    # Exit the loop by pressing 'q' key
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# Release the video capture object and destroy all windows
vid.release()
cv2.destroyAllWindows()
