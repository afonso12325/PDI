import vrep
import cv2
import array
import numpy as np
import time
from PIL import Image as I
from math import atanh
class Initializer():
    def __init__(self,address = '127.0.0.1', port = 19997):
        print('program started')
        vrep.simxFinish(-1)
        self.clientID=vrep.simxStart(address,port,True,True,5000,5)
        print ('Connected to remote API server')
        
class Robot():
    def __init__(self, clientID,left_motor = "Pioneer_p3dx_leftMotor", right_motor="Pioneer_p3dx_rightMotor"):
        r, self.leftmotor = vrep.simxGetObjectHandle(clientID, left_motor, vrep.simx_opmode_oneshot_wait);
        r, self.rightmotor = vrep.simxGetObjectHandle(clientID, right_motor, vrep.simx_opmode_oneshot_wait);
        self.clientID = clientID
        vrep.simxSetJointTargetVelocity(clientID, self.leftmotor, 0, vrep.simx_opmode_streaming);
        vrep.simxSetJointTargetVelocity(clientID, self.rightmotor, 0, vrep.simx_opmode_streaming);
    def set_motors(self, vl = 1,vr = 1):
        vrep.simxSetJointTargetVelocity(self.clientID, self.leftmotor, vl, vrep.simx_opmode_streaming);
        vrep.simxSetJointTargetVelocity(self.clientID, self.rightmotor, vr, vrep.simx_opmode_streaming);

class Camera():
    def __init__(self, clientID, camera = "kinect_rgb"):
        self.clientID = clientID
        r, self.colorCam = vrep.simxGetObjectHandle(clientID, camera , vrep.simx_opmode_oneshot_wait);
        r, self.resolution, image = vrep.simxGetVisionSensorImage(clientID, self.colorCam, 1, vrep.simx_opmode_streaming);
        time.sleep(0.5)
    def shot(self):
            r, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, self.colorCam, 1, vrep.simx_opmode_buffer);
            mat = np.asarray(image, dtype=np.uint8) 
            mat2 = mat.reshape(resolution[1], resolution[0], 1)
            img = cv2.flip( mat2, 0 )
            return img
    def process(self,img):

        #img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(img, 0, 30)
        mask = cv2.medianBlur(mask, 5)
        roi = mask[int(29*mask.shape[0]/30):mask.shape[0],:]	
        cv2.imshow('mask', mask)
        sumx=0
        sumy=0
        l=0
        for x in range(roi.shape[0]):
            for y in range(roi.shape[1]):
                if roi[x,y]>0:
                    sumy+=x
                    sumx+=y
                    l+=1
        if(l>0.01*roi.shape[0]*roi.shape[1]):
            centroid = (sumx/l-roi.shape[1]/2)/(roi.shape[1]/2)
        else:
            centroid = None
        return centroid
class Controller():
    def __init__(self, clientID, robot):
        self.ki =0
        self.kp = .75
        self.kd = .4
        self.I = 0
        self.clientID = clientID
        self.robot = robot
    def control_pid(self, cent, last_cent, dt):
        P = self.kp*cent
        self.I += self.ki*(cent)
        D = self.kd*((cent)-(last_cent))
        PID = P+self.I+D
        print(PID)
        self.robot.set_motors(2+PID*2, 2-PID*2)
    def control(self,cent_x):
            if abs(cent_x)<50:
                self.robot.set_motors(0.5,0.5)
            elif cent_x>0:
                self.robot.set_motors(0.5,-0.5)
            elif cent_x<0:
                self.robot.set_motors(-0.5,0.5)
            else:
                print("no centroid found")
                self.robot.set_motors(0.5,-0.5)
