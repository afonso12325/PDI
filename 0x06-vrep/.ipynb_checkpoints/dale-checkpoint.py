import vrep
import cv2
import array
import numpy as np
import time
from PIL import Image as I
from vrepclasses import Initializer, Robot, Camera, Controller

init = Initializer()
robot = Robot(init.clientID)
camera = Camera(init.clientID)
controller = Controller(init.clientID, robot)
centroid = None
dt = None
while True:
    start = time.time()
    img = camera.shot()
    cent_ant = centroid
    centroid = camera.process(img)

    if cent_ant is not None and centroid is not None:
        controller.control_pid(centroid, cent_ant, dt)
    else:
        pass
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    end = time.time()
    dt = end - start

vrep.simxStopSimulation(init.clientID, vrep.simx_opmode_oneshot)
