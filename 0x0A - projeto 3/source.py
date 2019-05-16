#!/usr/bin/python
# -*- coding: utf-8 -*-

''' VIDEOS MUTS BE IN FOLDER videos/ AND MASKS IN FOLDER masks/ '''

import argparse
import cv2
import numpy as np
from os import listdir
from os.path import isfile, join
import re
import os

drawing = False  # true if mouse is pressed
erasing = False
mode = 'inpaint'  # if True, draw rectangle. Press 'm' to toggle to curve
(ix, iy) = (-1, -1)
(lix, liy) = (-1, -1)
l_widht = 3
frame = None
img = None


def draw_circle(
    event,
    x,
    y,
    flags,
    param,
    ):
    '''mouse callback function'''

    global ix, iy, drawing, mode, lix, liy, erasing, l_widht

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        (lix, liy) = (x, y)
        (ix, iy) = (x, y)
    if event == cv2.EVENT_RBUTTONDOWN:
        erasing = True
        (lix, liy) = (x, y)
        (ix, iy) = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE:

        if drawing == True and lix >= 0 and liy >= 0:
            cv2.line(img, (lix, liy), (ix, iy), 0xFF, l_widht)
            (lix, liy) = (ix, iy)
            (ix, iy) = (x, y)
        if erasing == True and lix >= 0 and liy >= 0:
            cv2.line(img, (lix, liy), (ix, iy), 0, l_widht)
            (lix, liy) = (ix, iy)
            (ix, iy) = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        (lix, liy) = (-1, -1)
    elif event == cv2.EVENT_RBUTTONUP:
        erasing = False
        (lix, liy) = (-1, -1)


def get_name(dir_path):
    '''matches file name'''
    if dir_path[:7] != 'videos/':
        raise Exception('videos must be in "videos/" directory') 

    searchObj = re.search(r'\/(\w+)\.', dir_path, re.M | re.I)

    return searchObj.group(1)


def check_name(name, path='masks/'):
    '''check if exists a mask with name'''
    if not os.path.exists(path):
         os.mkdir('masks/')
    onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]
    return not 'mask-' + name + '.npy' in onlyfiles


def draw(dir_path):
    '''draw mask'''

    global img, frame, l_widht
    name = get_name(dir_path)

    capture = cv2.VideoCapture(dir_path)
    capture.set(cv2.CAP_PROP_FPS, 8)

    if check_name(name):
        while frame is None:
            (ret, frame) = capture.read()

        frame_cp = frame.copy()
        img = np.zeros(frame.shape[:2] + (1, ), np.uint8)
        cv2.namedWindow('image')
        cv2.namedWindow('mask')
        cv2.setMouseCallback('image', draw_circle)
        cv2.setMouseCallback('mask', draw_circle)

        while 1:
            cv2.imshow('image', cv2.bitwise_or(frame, frame_cp,
                       mask=cv2.bitwise_not(img)))
            cv2.imshow('mask', img)
            k = cv2.waitKey(1) & 0xFF
            if k == 43 and l_widht < 15:
                l_widht += 1
            if k == 45 and l_widht > 1:
                l_widht -= 1
            if k == 27:
                break
        np.save('masks/mask-' + name + '.npy', img)
    else:

        img = np.load('masks/mask-' + name + '.npy')
    cv2.destroyWindow('image')
    cv2.destroyWindow('mask')
    return capture


def show(capture):
    '''inpaint and show result'''

    global img, mode
    cv2.namedWindow('result')
    while 1:
        (ret, frame) = capture.read()
        if ret == True:
            if mode == 'inpaint':
                inpainted = cv2.inpaint(frame, img, 3,
                        cv2.INPAINT_TELEA)
            else:
                inpainted = frame
            cv2.imshow('result', inpainted)
            k = cv2.waitKey(1) & 0xFF

            if k == 27:
                break
            if k == ord('m'):
                if mode == 'inpaint':
                    mode = 'normal'
                else:
                    mode = 'inpaint'

    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='video inpaint')
    parser.add_argument('v', metavar='video_dir', help='video path')
    args = parser.parse_args()
    cap = draw(args.v)
    show(cap)
