#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,6,0)
objp = np.zeros((9*9,3), np.float32)                                           #attention dimension echiqier -1
objp[:,:2] = np.mgrid[0:9,0:9].T.reshape(-1,2)*0.02479                           #attention dimension echiqier -1
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('calibration_cam_bluerov/images/*.png')
for fname in images:
    img = cv.imread(fname)
    # CLAHE (Contrast Limited Adaptive Histogram Equalization)
    clahe = cv.createCLAHE(clipLimit=3., tileGridSize=(8,8))
    
    lab = cv.cvtColor(img, cv.COLOR_BGR2LAB)  # convert from BGR to LAB color space
    l, a, b = cv.split(lab)  # split on 3 different channels
    
    l2 = clahe.apply(l)  # apply CLAHE to the L-channel
    
    lab = cv.merge((l2,a,b))  # merge channels
    img = cv.cvtColor(lab, cv.COLOR_LAB2BGR)  # convert from LAB to BGR

    # Gamma correction
    lookUpTable = np.empty((1,256), np.uint8)
    gamma = 0.7                               ##corrige la brightness non linéairement
    for i in range(256):
        lookUpTable[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
    img = cv.LUT(img, lookUpTable)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,9), None)       #attention dimension echiqier -1
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (9,9), corners2, ret)              #attention dimension echiqier -1
        cv.imshow('img', img)
        cv.waitKey(500)
cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)

np.save("calibration_cam_bluerov/matrices/camera_matrix", mtx)
np.save("calibration_cam_bluerov/matrices/distortion_coeffs", dist)

