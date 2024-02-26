import cv2
import numpy as np

vid = cv2.VideoCapture("Videos/1.mp4")

while(1):
    _, img = vid.read()
    cv2.imshow("Video",img)
    if cv2.waitKey(30) &0xff == 27:
        break

cv2.destroyAllWindows()
