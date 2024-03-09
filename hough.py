import cv2
import numpy

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