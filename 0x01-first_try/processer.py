import cv2
import numpy as np

def processer(frame, frame_number):
    #do the processing
    
    
    
    processed = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    add_hist(processed)
    
    #show it
    cv2.putText(processed,'PROCESSED IMAGE', (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.imshow('frame',processed)

def default(frame, e):

    cv2.putText(frame,'DEFAULT IMAGE', (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (160,255,255), 2)
    cv2.putText(frame, str(e), (0,100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
    cv2.imshow('frame',frame)
    
def add_hist(img):
    h = cv2.calcHist([img],[0],None,[256],[0,256])
    for i, bin_ in enumerate(h):
        cv2.line(img,(i,img.shape[0]),(i,img.shape[0] - int(bin_*0.01)),(255,255,255))
    return h.reshape((-1)).T
    