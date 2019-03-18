import cv2
import numpy as np
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
orb = cv2.ORB_create() 
def processer(frame, last_frame, frame_number):
    #do the processing

    frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # Initiate SIFT detector
     
    if last_frame is not None:
        last_frame_gray = cv2.cvtColor(last_frame,cv2.COLOR_BGR2GRAY)
        kp1 = cv2.goodFeaturesToTrack(last_frame_gray, 10000, 0.01, 5)
        kp2 = cv2.goodFeaturesToTrack(frame_gray, 10000, 0.01, 5)
        kp1s = [cv2.KeyPoint(x[0][0],x[0][1], 20) for x in kp1]
        kp2s = [cv2.KeyPoint(x[0][0],x[0][1], 20) for x in kp2]

        # find the keypoints and descriptors with SIFT
        kp1, des1 = orb.compute(last_frame_gray,kp1s)
        kp2, des2 = orb.compute(frame_gray,kp2s)

        # Match descriptors.
        matches = bf.match(des1,des2)

        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x:x.distance)
        for match in matches[:10]:
            inicio = (int(kp1[match.queryIdx].pt[0]), int(kp1[match.queryIdx].pt[1]))
            fim = (int(kp2[match.trainIdx].pt[0]), int(kp2[match.trainIdx].pt[1]))
            cv2.circle(frame,fim,3, (0,0,255))
            cv2.line(frame, inicio, fim, (255,0,0))
        #show itS
    cv2.putText(frame,'PROCESSED IMAGE', (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.imshow('frame', frame)



    
