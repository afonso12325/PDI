import vrep
import cv2
import array
import numpy as np
import time
from PIL import Image as I

print('program started')
vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
print ('Connected to remote API server')
r, colorCam = vrep.simxGetObjectHandle(clientID, "kinect_rgb", vrep.simx_opmode_oneshot_wait);
r, leftmotor = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait);
r, rightmotor = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_oneshot_wait);

vrep.simxSetJointTargetVelocity(clientID, leftmotor, 0, vrep.simx_opmode_streaming);
vrep.simxSetJointTargetVelocity(clientID, rightmotor, 0, vrep.simx_opmode_streaming);

r, resolution, image = vrep.simxGetVisionSensorImage(clientID, colorCam, 1, vrep.simx_opmode_streaming);
time.sleep(0.5)


cor1 = (0, 0, 0)
cor2 = (180, 255, 30)

while True:
	r, resolution, image = vrep.simxGetVisionSensorImage(clientID, colorCam, 1, vrep.simx_opmode_buffer);
	mat = np.asarray(image, dtype=np.uint8) 
	mat2 = mat.reshape(resolution[1], resolution[0], 1)

	img = cv2.flip( mat2, 0 )
	#img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
	
	#mask = cv2.inRange(img_hsv, cor1, cor2)

	vrep.simxSetJointTargetVelocity(clientID, leftmotor, 1, vrep.simx_opmode_streaming);
	vrep.simxSetJointTargetVelocity(clientID, rightmotor, -1, vrep.simx_opmode_streaming);	
	cv2.imshow('robot camera', img)	

	if cv2.waitKey(1) & 0xFF == ord('q'):
            break

	cv2.waitKey(1)