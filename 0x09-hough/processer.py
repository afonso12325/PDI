import cv2
import numpy as np

def houghp(img):
    img = cv2.Canny(img, 50, 80)
    return (cv2.HoughLinesP(img, 1, np.pi / 180, 100, None, 0, 0), img)

def hough(img):
    img = cv2.Canny(img, 50, 80)
    return (cv2.HoughLines(img, 1, np.pi / 180, 100, None, 0, 0), img)