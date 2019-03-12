import cv2
import numpy as np
def processer(frame, frame_number):
    #do the processing
    
    

    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray,2,3,0.04)

    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    # Threshold for an optimal value, it may vary depending on the image.
    frame[dst>0.1*dst.max()]=[0,0,255]
    #show it
    cv2.putText(frame,'PROCESSED IMAGE', (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.imshow('frame',frame)

def default(frame, e):

    cv2.putText(frame,'DEFAULT IMAGE', (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (160,255,255), 2)
    cv2.putText(frame, str(e), (0,100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
    cv2.imshow('frame',frame)

    
