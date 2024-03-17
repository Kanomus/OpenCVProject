import cv2
import numpy as np

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
"""

def getlines(img, threshold, r_threshold, theta_threshold):
        height,width=img.shape

        diagonal_length=int(-numpy.sqrt((height**2)+(width**2)))
        r_range=range(diagonal_length, -diagonal_length, r_threshold)
        theta_range=numpy.deg2rad(numpy.arange(-90, 90, theta_threshold))

        lines=numpy.zeros((len(r_range), len(theta_range)), dtype=numpy.uint64)
        x,y = numpy.nonzero(img)

        for i in range (len(x)):
                for j in range(len(theta_range)):
                        r = int( (x[i]*numpy.cos(theta_range[j])) + (y[i]*numpy.sin(theta_range[j])) + diagonal_length)
                        lines[r,j]+=1

        return lines

def drawlines(img, lines, threshold, length, thickness):
    for r in range(len(lines)):
        for theta in range(len(lines[r])):
            if lines[r][theta] > threshold:
                rho = r
                theta_rad = numpy.deg2rad(theta)
                a = numpy.cos(theta_rad)
                b = numpy.sin(theta_rad)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + length * (-b))
                y1 = int(y0 + length * (a))
                x2 = int(x0 - length * (-b))
                y2 = int(y0 - length * (a))
                cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), thickness=thickness)
"""


