import cv2
import numpy as np

vid = cv2.VideoCapture("Videos/1.mp4")
subtractor = cv2.createBackgroundSubtractorKNN()

while(1):
    _, img = vid.read()
    feed=cv2.resize(img,(0,0),fx=0.5,fy=0.5)

    cv2.imshow("Video",subtractor.apply(feed))
    if cv2.waitKey(30) == 27:
        break
vid.release()
cv2.destroyAllWindows()
