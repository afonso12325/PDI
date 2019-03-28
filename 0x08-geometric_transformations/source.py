import math
import numpy as np
import cv2

width = 640*2
height = 480*2

referencePoints = [np.float32(
    [[width/4, height/4],
    [2*width/4, height/4], 
    [2*width/4, 2*height/4], 
    [width/4, 2*height/4]]
),
np.float32([[2.5*width/4, 2.5*height/4],
    [3*width/4, 2.5*height/4], 
    [3*width/4, 3*height/4], 
    [2.5*width/4, 3*height/4]]
    ),

np.float32([[3.5*width/4, 3.5*height/4],
    [4*width/4, 3.5*height/4], 
    [4*width/4, 4*height/4], 
    [3.5*width/4, 4*height/4]]
    )]

currentPoint = (-1, -1)
calibrating = True
fullScreen = False
ready = False
# TODO: mudar o nome do arquivo da imagem
inputImage1 = [cv2.imread("naruto.jpg"), cv2.imread("naruto.jpg"), cv2.imread('peppa.jpg')]
rows1, cols1 = [i.shape[0] for i in inputImage1], [i.shape[1] for i in inputImage1]
pts1 = [np.float32(
    [[0,0],
    [cols,0],
    [cols, rows],
    [0, rows]]
) for cols, rows in zip(cols1, rows1)]

image = np.zeros((height, width, 3), dtype=np.uint8)

def pointColor(n):
    if n==0:
        return (0, 0, 255)
    elif n==1:
        return (0, 255, 255)
    elif n==2:
        return (255, 255, 0)
    else:
        return (0, 255, 0)

def mouse(event, x, y, flags, param):
    global currentPoint

    if event == cv2.EVENT_LBUTTONDOWN:
        cp = 0
        for i in range(len(referencePoints)):
            for point in referencePoints[i]:
                dist = math.sqrt((x-point[0])*(x-point[0])+(y-point[1])*(y-point[1]))
                if dist < 10:
                    print("mouse event ponto {},{} indice {}".format(x,y,i))
                    currentPoint = (cp%4,i)
                    print(currentPoint[1])
                    break
                else:
                    cp += 1

    if event == cv2.EVENT_LBUTTONUP:
        currentPoint = (-1,-1)
    
    if currentPoint[0] != -1:
            referencePoints[currentPoint[1]][currentPoint[0]] = [x, y]
cv2.namedWindow("test", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("test", mouse)
cap = [cv2.VideoCapture('batman.mp4'),cv2.VideoCapture('morty.mp4'),cv2.VideoCapture('mandelbrot.mp4')]

while True:
    image[:] = (0, 0, 0)
    if calibrating:
        color = 0
        for i in range(len(referencePoints)):
            for point in referencePoints[i]:
                cv2.circle(image, (int(point[0]), int(point[1])),5, pointColor(color%4), -1)
                color += 1
    for i in range(len(referencePoints)):
        
        if not ready:
            M = cv2.getPerspectiveTransform(pts1[i], referencePoints[i])
            cv2.warpPerspective(inputImage1[i], M, (width, height), image, borderMode=cv2.BORDER_TRANSPARENT)
        else:
            frame = cap[i].read()[1]
            rows1, cols1 = frame.shape[0], frame.shape[1]
            pts1 = np.float32(
                [[0,0],
                [cols1,0],
                [cols1, rows1],
                [0, rows1]])
            
            M = cv2.getPerspectiveTransform(pts1, referencePoints[i])
            cv2.warpPerspective(frame, M, (width, height), image, borderMode=cv2.BORDER_TRANSPARENT)
    cv2.imshow("test", image)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("r"):
        ready = True
    if key == ord("c"):
        calibrating = not calibrating
    
    if key == ord("f"):
        if not fullScreen:
            cv2.setWindowProperty("test", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else:
            cv2.setWindowProperty("test", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        fullScreen = not fullScreen
    
    if key == ord("q"):
        break

cv2.destroyAllWindows()