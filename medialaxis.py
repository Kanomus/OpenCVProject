import cv2
import numpy as np
import edgedetection
import os

my_path = os.path.abspath(os.path.dirname(__file__))
path = os.path.join(my_path, "Videos/1.mp4")
vid = cv2.VideoCapture(path)              #video input

subtractor= cv2.createBackgroundSubtractorKNN()     #creating subtractor object
kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,5))

while(1):
    _, img = vid.read()
    img=cv2.resize(img,(0,0),fx=0.5,fy=0.5)
    img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    fg=subtractor.apply(img)                       #applying subtractor to theimage frame
    
    #fg=cv2.erode(fg,kernel,iterations=1)            #applying morphological tranforms to clean up the image

    feed=edgedetection.detectedges(fg,padding=1,stride=1,threshold=100)

    cv2.imshow("Video",feed)
    if cv2.waitKey(30) == 27:
        break

cv2.destroyAllWindows()
