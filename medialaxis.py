import cv2
import numpy as np
import edgedetection
import os
import hough

my_path = os.path.abspath(os.path.dirname(__file__))
filenumber = str(input("Which video to open? (1, 2, or 3):\n"))

path = os.path.join(my_path, "Videos",filenumber+".mp4")
vid = cv2.VideoCapture(path)              #video input

subtractor= cv2.createBackgroundSubtractorKNN()     #creating subtractor object
kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,5))

while(1):
    _, img = vid.read()
    img=cv2.resize(img,(0,0),fx=0.5,fy=0.5)
    bw_img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    fg=subtractor.apply(bw_img)                       #applying subtractor to theimage frame
    
    #fg=cv2.erode(fg,kernel,iterations=1)            #applying morphological tranforms to clean up the image

    feed=edgedetection.detectedges(fg,padding=1,stride=1,threshold=100)

    lines=hough.hough_lines(feed, rho_resolution=2, theta_resolution=6, threshold=100)
    groupedlines=hough.group_lines_by_angle(lines)
    longestgroup=hough.find_longest_group(grouped_lines=groupedlines)

    hough.draw_mean_line(img, longestgroup)
    '''lines=hough.getlines(feed, threshold=0, theta_threshold=1, r_threshold=1)
    hough.drawlines(img, lines, threshold=50, length=5, thickness=1)
    '''

    cv2.imshow("Video",img)
    if cv2.waitKey(30) == 27:
        break

cv2.destroyAllWindows()
