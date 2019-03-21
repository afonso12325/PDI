import vrep
import cv2
import array
import numpy as np
import time
from PIL import Image as I
from math import atanh
def sigmoid(x):
    return 1. / ( 1. + np.exp(-x) )
def mean(x,y):
    return (x+y)/2
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
        d_roi = mask[int(29*mask.shape[0]/30):mask.shape[0],:].copy()
        m_roi = mask[int(14*mask.shape[0]/30):int(15*mask.shape[0]/30)].copy()
        u_roi = mask[int(3*mask.shape[0]/30):int(4*mask.shape[0]/30)].copy()
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
#                                    lower centroid
        whites = np.argwhere(d_roi>0)
        if(len(whites)>0.01*d_roi.shape[0]*d_roi.shape[1]):
            unormalized_centroids = np.mean(whites.T, axis = 1)
            cv2.circle(mask, (int(unormalized_centroids[1]),int(unormalized_centroids[0])+ int(29*mask.shape[0]/30)), 3, (0,0,255))
            d_centroid = (2*unormalized_centroids[1] - d_roi.shape[1])/d_roi.shape[1]
        else:
            d_centroid = None
#                                    middle centroid
        whites = np.argwhere(m_roi>0)
        if(len(whites)>0.01*m_roi.shape[0]*m_roi.shape[1]):
            unormalized_centroids = np.mean(whites.T, axis = 1)
            cv2.circle(mask, (int(unormalized_centroids[1]),int(unormalized_centroids[0])+ int(14*mask.shape[0]/30)), 3, (255,0,0))
            m_centroid = (2*unormalized_centroids[1] - m_roi.shape[1])/m_roi.shape[1]
        else:
            m_centroid = None
        
#                                    upper centroid


        whites = np.argwhere(u_roi>0)
        if(len(whites)>0.01*u_roi.shape[0]*u_roi.shape[1]):
            unormalized_centroids = np.mean(whites.T, axis = 1)
            cv2.circle(mask, (int(unormalized_centroids[1]),int(unormalized_centroids[0])+ int(3*mask.shape[0]/30)), 3, (0,255,0))
            u_centroid = (2*unormalized_centroids[1] - u_roi.shape[1])/u_roi.shape[1]
        else:
            u_centroid = None
        





        cv2.imshow('mask', mask)
        return (d_centroid, m_centroid, u_centroid)
class Controller():
    def __init__(self, clientID, robot):
        #fast?
#         self.F = 1
#         self.v_factor = 0
#         self.ki =0
#         self.kp = .7
#         self.kd = .3
#         self.I = 0
#         self.ki2 =0
#         self.kp2 = 1
#         self.kd2 = .7
#         self.I2 = 0
#         self.ki3 =0
#         self.kp3 = 1
#         self.kd3 = .7
#         self.I3 = 0


        self.clientID = clientID
        self.robot = robot
#
        self.F = 1/2
        self.v_factor = 0
        self.ki =0
        self.kp = .7
        self.kd = .3
        self.I = 0
    def control_pid(self, d_cent, last_d_cent, dt):
        P = self.kp*d_cent
        self.I += self.ki*self.F*(d_cent)
        D = self.kd*((d_cent)-(last_d_cent))/(self.F)
        PID = P+self.I+D
        self.robot.set_motors(self.F*(2+PID*4), self.F*(2-PID*4))
    def control_pid_2(self, d_cent, last_d_cent, u_cent, last_u_cent, dt):
        P = self.kp*d_cent
        self.I += self.ki*self.F*(d_cent)
        D = self.kd*((d_cent)-(last_d_cent))/self.F
        PID = P+self.I+D
        
        
        P2 = self.kp2*atanh(u_cent)
        self.I2 += self.ki2*(u_cent)*self.F
        D2 = self.kd2*((u_cent)-(last_u_cent))/self.F
        PID2 = P2+self.I2+D2
        #fast?
#         self.vfactor = 1.5*sigmoid(1-mean(u_cent,d_cent))
        
        
        TPID = 0.7*PID+0.3*PID2
        
        v_left = self.F*(2+TPID*4+2*self.vfactor)
        v_right = self.F*(2-TPID*4+2*self.v_factor) 
        #print('v_left = {}\n v_right = {}'.format(v_left,v_right))
        print('v_left+v_right = {}'.format(v_left+v_right))
        self.robot.set_motors(v_left, v_right)
        
    def control_pid_3(self, d_cent, last_d_cent, m_cent, last_m_cent,u_cent, last_u_cent, dt):
        P = self.kp*d_cent
        self.I += self.ki*self.F*(d_cent)
        D = self.kd*((d_cent)-(last_d_cent))/self.F
        PID = P+self.I+D
        
        
        P2 = self.kp2*atanh(m_cent)
        self.I2 += self.ki2*(m_cent)*self.F
        D2 = self.kd2*atanh((m_cent)-(last_m_cent))/self.F
        PID2 = P2+self.I2+D2
        
        P3 = self.kp3*atanh(u_cent)
        self.I3 += self.ki3*(u_cent)*self.F
        D3 = self.kd3*atanh((u_cent)-(last_u_cent))/self.F
        PID3 = P3+self.I3+D3
        
        self.vfactor = 1.5*sigmoid(1-mean(m_cent,d_cent))
        TPID = 0.7*PID+0.2*PID2+0.1*PID3
        
        v_left = self.F*(2+TPID*4+2*self.vfactor)
        v_right = self.F*(2-TPID*4+2*self.v_factor) 
        #print('v_left = {}\n v_right = {}'.format(v_left,v_right))
        print('v_left+v_right = {}'.format(v_left+v_right))
        self.robot.set_motors(v_left, v_right)
        
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
