import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import processer
from importlib import reload
cap = cv2.VideoCapture(0)



while(1):
    ret, img = cap.read()
    img = cv2.resize(img, (320,240))
    img2 = img.copy()
    if ret:

        linhas_P, img = processer.houghp(img)
        img_copia = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if linhas_P is not None:
            for linha in linhas_P:
                print(linha)
                x1, y1, x2, y2 = linha[0]
                cv2.line(img_copia, (x1, y1), (x2, y2), (0, 0, 255), 3)

        
        linhas, img2 = processer.hough(img2)
        img_copia2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
        if linhas is not None:
            for i in range(0, len(linhas)):
                rho = linhas[i][0][0]
                theta = linhas[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(img_copia2, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)







            reload(processer)

            cv2.imshow('image2',img_copia2)
            cv2.imshow('image',img_copia)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('m'):
        mode = not mode
    elif k == ord('q'):
        break

cv2.destroyAllWindows()