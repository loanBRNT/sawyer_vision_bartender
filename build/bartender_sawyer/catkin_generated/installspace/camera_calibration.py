#!/usr/bin/env python3

import rospy
import intera_interface
import argparse
import cv2
import numpy as np
import os
import glob

CHECKERBOARD = (6, 9)

criteria = (CV2.TERM_CRITERA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

threedpoints = []

twodpoints = []

objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)

prev_img_shape = None

camera ='right_hand_camera'

def calibration():
	rospy.init_node('camera_calibration')
	
	images = glob.glob('./img/Calibration/*')
	
	print(images)
	

if __name__ == '__main__':
	try:
		calibration()
	except rospy.ROSInterruptException:
		pass