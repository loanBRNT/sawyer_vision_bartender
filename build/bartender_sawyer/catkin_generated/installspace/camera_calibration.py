#!/usr/bin/env python3

import rospy
import intera_interface
import argparse
import cv2
import numpy as np
import os
import glob

CHECKERBOARD = (6, 5)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

threedpoints = []

twodpoints = []

objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)

prev_img_shape = None

camera ='right_hand_camera'

def calibration():
	rospy.init_node('camera_calibration')
	
	images = glob.glob('./img/Calibration/*')
	
	for f in images:
		img = cv2.imread(f)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		
		black = np.zeros(img.shape, dtype=np.uint8)
		img = cv2.addWeighted(img, 1, black, 0, 50)
		
		ret, corners = cv2.findChessboardCorners(img, 
			CHECKERBOARD,cv2.CALIB_CB_ADAPTIVE_THRESH 
			+ cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
		
		print(ret)
		
		if ret == True:
			threedpoints.append(objectp3d)
			
			corners2 = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
			
			twodpoints.append(corners2)
			
			img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
		
		cv2.imshow('img',img)
		cv2.waitKey(0)
		
	cv2.destroyAllWindows()
	
	
	ret, matrix, distorsion, r_vecs, t_vecs = cv2.calibrateCamera(threedpoints, twodpoints, img.shape[::-1],None,None)
	
	print("Camera matrix:")
	print(matrix)

if __name__ == '__main__':
	try:
		calibration()
	except rospy.ROSInterruptException:
		pass
