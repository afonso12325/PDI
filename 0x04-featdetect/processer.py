import cv2
import numpy as np
def processer(frame, frame_number):
    #do the processing

    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,20,220)
    #show it
    cv2.putText(frame,'PROCESSED IMAGE', (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.imshow('frame',edges)


    
