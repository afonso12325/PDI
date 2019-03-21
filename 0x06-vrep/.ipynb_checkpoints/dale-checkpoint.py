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
d_centroid = None
m_centroid = None
u_centroid = None
dt = None
while True:
    start = time.time()
    img = camera.shot()
    d_cent_ant = d_centroid
    d_centroid = camera.process(img)[0]
    m_cent_ant = m_centroid
    m_centroid = camera.process(img)[1]
    u_cent_ant = u_centroid
    u_centroid = camera.process(img)[2]
#     #fast?
#     if d_cent_ant is not None and d_centroid is not None:
#         if m_cent_ant is not None and m_centroid is not None:
#             if u_cent_ant is not None and u_centroid is not None:
#                 controller.control_pid_3(d_centroid, d_cent_ant,m_centroid, m_cent_ant , u_centroid, u_cent_ant, dt)
#             else:
#                 controller.control_pid_2(d_centroid, d_cent_ant,m_centroid, m_cent_ant , dt)
#         else:
#             controller.control_pid(d_centroid, d_cent_ant, dt)
#     else:
#         pass
#precise?
    if d_cent_ant is not None and d_centroid is not None:
        controller.control_pid(d_centroid, d_cent_ant,dt)
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    end = time.time()
    dt = end - start

vrep.simxStopSimulation(init.clientID, vrep.simx_opmode_oneshot)


