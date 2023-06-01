#!/usr/bin/env python3

import rospy
import intera_interface
import argparse
import numpy as np
import cv2
import torch
from cv_bridge import CvBridge, CvBridgeError

from detection import DetectionYolov5

def show_image_callback(img_data, xxx_todo_changeme):
    """The callback function to show image by using CvBridge and cv
    """
    (edge_detection, window_name) = xxx_todo_changeme
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError as err:
        rospy.logerr(err)
        return
    cv2.namedWindow(window_name, 0)
    # refresh the image on the screen
    cv2.imshow(window_name, cv_image)
    cv2.waitKey(3)

def glass_detection():

    rp = intera_interface.RobotParams()
    cam = 'right_hand_camera'

    parser = argparse.ArgumentParser()
    #parser.add_argument('-c', '--camera', type=str, default="head_camera",help='Setup Camera Name for Camera Display')
    args = parser.parse_args()

    print("Initializing node... ")
    rospy.init_node('glass_detection')
    cameras = intera_interface.Cameras()

    cameras.start_streaming(cam)

    cameras.set_callback(cam, show_image_callback,
        rectify_image=True, callback_args=(False, cam))

    cameras.set_exposure(cam, 5)
    
    d = DetectionYolov5()


    def clean_shutdown():
        print("Shutting down")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera running. Ctrl-c to quit")
    rospy.spin()

if __name__ == '__main__':
	try:
		glass_detection()
	except rospy.ROSInterruptException:
		pass
