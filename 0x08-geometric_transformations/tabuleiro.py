 
import numpy as np
import cv2


width = 640
height = 640
sqx = int(width/8)
sqy = int(height/8)
tabuleiro  = np.zeros((height, width, 3), dtype=np.uint8)
referencePoints = []
frame = cv2.imread('naruto.jpg')
for y in range(8):
    for x in range(8):
        if (x+y)%2 == 0:
            tabuleiro[x*sqx:(x+1)*sqx,y*sqy:(y+1)*sqy] = 255
            referencePoints.append(np.float32([[x*sqx, y*sqy], [(x+1)*sqx,y*sqy], [(x)*sqx,(y+1)*sqy], [(x+1)*sqx,(y+1)*sqy]]

            ))
cap = cv2.VideoCapture("morty.mp4")
cap = cv2.VideoCapture(0)
frame = None
while frame is None:
    frame = cap.read()[1]
    print("oi")

rows1, cols1 = frame.shape[0], frame.shape[1]
pts1 = np.float32(
                [[0,0],
                [cols1,0],[0, rows1],
                [cols1, rows1]
                ])

M = [cv2.getPerspectiveTransform(pts1, rp) for rp in referencePoints]
i = 0
while True:

    frame = cap.read()[1]
    cv2.warpPerspective(frame, M[i%32], (width, height), tabuleiro, borderMode=cv2.BORDER_TRANSPARENT)
    cv2.imshow("tab", tabuleiro)





    i+=1
    key = cv2.waitKey(1) & 0xFF
    if key == ord("f"):
        if not fullScreen:
            cv2.setWindowProperty("test", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else:
            cv2.setWindowProperty("test", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        fullScreen = not fullScreen
    
    if key == ord("q"):
        break

cv2.destroyAllWindows()