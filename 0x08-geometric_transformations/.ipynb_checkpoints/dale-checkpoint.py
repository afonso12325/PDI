import cv2
import numpy as np
import matplotlib.pyplot as plt
width = 640
height = 480
from screeninfo import get_monitors

for m in get_monitors():
    width, height = m.width, m.height
referencePoints = np.float32(
    [[width/4, height/4],
    [3*width/4, height/4], 
    [3*width/4, 3*height/4], 
    [width/4, 3*height/4]]
)
fundo = np.zeros((height, width, 3))
def draw_circle(event,x,y,flags,param):
    global pts1, mode

    if event == cv2.EVENT_LBUTTONDOWN:
        if mode == 1:
            pts1.append([x,y])
        elif mode == 2:
            pts3.append([x,y])
        print(type(x))



mode = 0
for point in referencePoints:
    cv2.circle(fundo,(point[0], point[1]), 4, (0,0,255))
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)
peppa = plt.imread('peppa.jpg')
pele = plt.imread('deboka.jpeg')
rows, cols = pele.shape[:2]
pts1 = []
pts2 = np.float32([[0,0],[cols,0],[0,rows],[cols,rows]])
peppa = cv2.cvtColor(peppa, cv2.COLOR_RGB2BGR)
pele =  cv2.cvtColor(pele, cv2.COLOR_RGB2BGR)
mode+=1
while(1):
    k = cv2.waitKey(1) & 0xFF
    cv2.imshow('fundo', fundo)
    cv2.setWindowProperty("fundo", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    cv2.imshow('image', peppa)
    if k == ord("q") or len(pts1)>=4:
        break

M = cv2.getPerspectiveTransform(pts2,np.float32(pts1))
r =cv2.warpPerspective(pele, M, (peppa.shape[1],peppa.shape[0]), peppa, borderMode=cv2.BORDER_TRANSPARENT)
pts3 = []
mode+=1
while(1):
    k = cv2.waitKey(1) & 0xFF
    cv2.imshow('image', peppa)
    if k == ord("q") or len(pts3)>=4:
        break
cap = cv2.VideoCapture(0)

frame = None
while frame is None:
    ret, frame = cap.read()
rows,cols = frame.shape[:2]
pts4 = np.float32([[0,0],[cols,0],[0,rows],[cols,rows]])
M = cv2.getPerspectiveTransform(pts4,np.float32(pts3))
print('here')
while(1):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    r =cv2.warpPerspective(frame,M,(peppa.shape[1],peppa.shape[0]), peppa, borderMode=cv2.BORDER_TRANSPARENT)
    cv2.imshow('image', peppa)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('m'):
        mode = not mode
    elif k == ord('q'):
        break
cv2.destroyAllWindows()