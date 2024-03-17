import cv2
import numpy as np
import sobeledge
import os
import hough

# Define video file path
my_path = os.path.abspath(os.path.dirname(__file__))
filenumber = str(input("Which video to open? (1, 2, or 3):\n"))

path = os.path.join(my_path, "Videos",filenumber+".mp4")
vid = cv2.VideoCapture(path)              #video input

if not vid.isOpened():
    print("Error opening video:", path)
    exit()

subtractor = cv2.createBackgroundSubtractorKNN()

while True:
    ret, img = vid.read()

    if not ret:
        print("Error reading frame from video:")
        break

    resized_img = cv2.resize(img, None, fx=0.5, fy=0.5)
    foreground_mask = subtractor.apply(resized_img)
    eroded_mask = cv2.erode(foreground_mask, np.ones((3, 3), np.uint8), iterations=2)
    edges = sobeledge.sobel_edge_detection(eroded_mask)
    new_img = np.zeros_like(resized_img)

    # Fine-tune the threshold for better line detection
    lines = hough.hough_lines(edges, rho_resolution=1, theta_resolution=np.pi/180, threshold=50)
    
    # Group lines by angle
    grouped_lines = hough.group_lines_by_angle(lines)
    
    # Find the longest continuous group of lines
    longest_group = hough.find_longest_group(grouped_lines)
    
    # Draw the mean line of the longest group
    hough.draw_mean_line(resized_img, longest_group)
    
    cv2.imshow("Detected Mean Line", resized_img)


    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
