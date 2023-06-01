#!/usr/bin/env python

import rospy
import intera_interface
import argparse
import numpy as np
import cv2
import torch
from time import time
#import pandas, tqdm, ultralytics
from cv_bridge import CvBridge, CvBridgeError

class DetectionYolov5:
	"""
	Class implements my trained model to detect cup and bottle. By Loan BERNAT.
	Github link : https://github.com/loanBRNT/sawyer_vision_bartender
	"""
	def __init__(self,model_path):
		self.model = self.load_model(model_path)
		self.classes = self.model.names
		self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
		rospy.loginfo("Using device '{0}'".format(self.device))
		
	def load_model(self,model_path):
		return torch.hub.load('ultralytics/yolov5','custom', path=model_path, force_reload=True)
		
	def score_frame(self,frame):
		self.model.to(self.device)
		results = self.model([frame])
		labels, cord = results.xyxyn[0][:,-1], results.xyxyn[0][:,:-1]
		return labels, cord
		
	def class_to_label(self, x):
		return self.classes[int(x)]
		
	def plot_boxes(self, results, frame):
		labels, cord = results
		n = len(labels)
		x_shape, y_shape = frame.shape[1], frame.shape[0]
		for i in range(n):
			row = cord[i]
			if row[4] >= 0.7:
				x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
				bgr = (0, 0, 255)
				cv2.rectangle(frame, (x1,y1), (x2,y2), bgr, 2)
				cv2.putText(frame, self.class_to_label(labels[i]), (x1,y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr)
		return frame
	
	def __call__(self,frame):
		frame = cv2.resize(frame, (640,640))
		
		start = time()
		
		results = self.score_frame(frame)
		rospy.loginfo(results)
		
		end = time()
		rospy.loginfo("Temps d'exec : '{0}'".format(end-start))
		cv2.namedWindow('detection', 0)
		cv2.imshow('detection', frame)
		cv2.waitKey(3)
		
#[INFO] [1685606991.576636]: Temps d'exec : '0.9965267181396484'
#[INFO] [1685606992.595954]: (tensor([0., 0.]), tensor([[0.40105, 0.35511, 0.70320, 0.93421, 0.75446],[0.65591, 0.07500, 0.97888, 0.69733, 0.32177]]))
detection = DetectionYolov5('/home/loan/sawyer_vision_bartender/model/best.pt')

def show_image_callback(img_data):
    """The callback function to show image by using CvBridge and cv
    """
    global detection

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
        detection(cv_image)
    except CvBridgeError as err:
        rospy.logerr(err)
        return

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
        rectify_image=True)

    cameras.set_exposure(cam, 5)

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
