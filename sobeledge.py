import cv2
import numpy as np

def sobel_edge_detection(img, ksize=3, threshold=127):
    sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
    sobel_y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])

    #image gradients
    gx = cv2.filter2D(img, cv2.CV_64F, sobel_x)
    gy = cv2.filter2D(img, cv2.CV_64F, sobel_y)

    #gradient magnitude and direction 
    magnitude = np.sqrt(gx**2 + gy**2)
    direction = np.arctan2(gy, gx)

    # Apply thresholding
    edge_map = np.where(np.abs(gx) + np.abs(gy) > threshold, 255, 0)  
    # Convert to binary 
    edge_map = edge_map.astype(np.uint8)

    return edge_map