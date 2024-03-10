import cv2
import numpy as np
import sobeledge

def hough_lines(edges, rho_resolution=1, theta_resolution=1, threshold=100):
    height, width = edges.shape
    max_rho = int(np.sqrt(height**2 + width**2))
    thetas = np.linspace(-np.pi / 2, np.pi / 2, int(np.pi / theta_resolution) + 1)
    accumulator = np.zeros((max_rho, len(thetas)), dtype=np.uint8)

    for y in range(height):
        for x in range(width):
            if edges[y, x] == 255:
                for theta in thetas:
                    rho = x * np.cos(theta) + y * np.sin(theta)
                    rho_index = int(np.round(rho / rho_resolution))
                    if 0 <= rho_index < max_rho:
                        accumulator[rho_index, int((theta + np.pi / 2) / theta_resolution)] += 1

    peaks = np.where(accumulator >= threshold)
    lines = [(rho_index * rho_resolution, thetas[theta_index]) for rho_index, theta_index in zip(*peaks)]
    return lines

def group_lines_by_angle(lines, theta_tolerance=np.pi/36):
    grouped_lines = []
    for rho, theta in lines:
        # Check if there's any group that this line belongs to based on angle
        found_group = False
        for group in grouped_lines:
            if abs(group[0][1] - theta) < theta_tolerance:
                group.append((rho, theta))
                found_group = True
                break
        if not found_group:
            grouped_lines.append([(rho, theta)])
    return grouped_lines

def find_longest_group(grouped_lines):
    longest_group = []
    max_length = 0
    for group in grouped_lines:
        if len(group) > max_length:
            longest_group = group
            max_length = len(group)
    return longest_group

def draw_mean_line(frame, lines):
    if lines:
        # Calculate mean line
        mean_rho = np.mean([rho for rho, _ in lines])
        mean_theta = np.mean([theta for _, theta in lines])

        a = np.cos(mean_theta)
        b = np.sin(mean_theta)
        x0 = a * mean_rho
        y0 = b * mean_rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

# Define video file path
video_path = "Videos/1.mp4"
vid = cv2.VideoCapture(video_path)

if not vid.isOpened():
    print("Error opening video:", video_path)
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
    lines = hough_lines(edges, rho_resolution=1, theta_resolution=np.pi/180, threshold=50)
    
    # Group lines by angle
    grouped_lines = group_lines_by_angle(lines)
    
    # Find the longest continuous group of lines
    longest_group = find_longest_group(grouped_lines)
    
    # Draw the mean line of the longest group
    draw_mean_line(resized_img, longest_group)
    
    cv2.imshow("Edges", edges)
    cv2.imshow("Detected Mean Line", resized_img)


    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
