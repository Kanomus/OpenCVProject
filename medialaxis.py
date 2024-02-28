import cv2
import numpy as np

vid = cv2.VideoCapture("Videos/1.mp4")              #video input

subtractor= cv2.createBackgroundSubtractorKNN()     #creating subtractor object
kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,5))

while(1):
    _, img = vid.read()
    feed=cv2.resize(img,(0,0),fx=0.5,fy=0.5)
    fg=subtractor.apply(feed)                       #applying subtractor to theimage frame
    
    fg=cv2.erode(fg,kernel,iterations=1)            #applying morphological tranforms to clean up the image

    cv2.imshow("Video",fg)
    if cv2.waitKey(30) == 27:
        break

cv2.destroyAllWindows()
