#!/usr/bin/env python3

import rospy
import intera_interface
import numpy as np
import cv2
import torch
import threading
import time
import argparse

from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath

#import pandas, tqdm, ultralytics
from cv_bridge import CvBridge, CvBridgeError

################ POSITION ###############

pos_detect_bottle = {'right_j0': 1.5445498046875, 'right_j1': -0.066890625, 'right_j2': -0.0502783203125, 'right_j3': 1.4486123046875, 'right_j4': 0.0872958984375, 'right_j5': -2.978140625, 'right_j6': 0.2274365234375}

pos_gen = {'right_j0': 1.61329296875, 'right_j1': 0.1555595703125, 'right_j2': -0.5116181640625, 'right_j3': 1.1607470703125, 'right_j4': 0.104712890625, 'right_j5': -1.26212109375, 'right_j6': -0.877208984375}

parser = argparse.ArgumentParser()
parser.add_argument('-d','--drink', type=str, help='Drink to pour')
parser.add_argument('-do','--dropOff', type=int, help='Position numbers (between 1 and 3) where the glass is')

wpt_opts = MotionWaypointOptions(max_linear_speed=0.6,
                                         max_linear_accel=0.6,
                                         max_rotational_speed=1.57,
                                         max_rotational_accel=1.57,
                                         max_joint_speed_ratio=1.0,
                                         corner_distance=0.1)
                                         
coefs_boissons={'Coca':1.25,'Orange':1.15,'Water':1.428}

################ CLASS ###############

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
		
		start = time.time()
		
		results = self.score_frame(frame)
		
		end = time.time()
		
		return results
		
		

################ FUNCTIONS ###############

detection = None
tab = []

def show_image_callback(img_data):
    """The callback function to show image by using CvBridge and cv
    """
    global detection
    global tab

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8") 
        tab.append(detection(cv_image))
    except CvBridgeError as err:
        rospy.logerr(err)
        return
        
def init_detection():
    global detection
    
    detection = DetectionYolov5('/home/loan/sawyer_vision_bartender/model/best_small_complet.pt')

def extract_mean_coord(tab):
    tab_mean=[]
    tab_labels=[]
    j = len(tab)-1
    coef=0
    while j > len(tab)/2 :
    	labels, coord = tab[j]
    	n = len(labels)
    	coord = coord.clone()
    	for i in range(n):
    	    if coef == 0:
    	    	tab_mean.append(coord[i])
    	    	tab_labels.append(labels[i])
    	    if n == len(tab_mean):
    	    	if tab_labels[i] == labels[i]:
	    	    	for k in range(5):
	    	    		tab_mean[i][k]= (tab_mean[i][k] * coef + coord[i][k]) / (coef + 1)
    	j-=1
    	coef+=1
    return tab_mean, tab_labels
    
def coord_from_best(tab_mean,tab_labels,boisson):

    global detection
    
    if not tab_mean:
    	return None, None, None, None, 0

    indice=-1
    
    for i in range(len(tab_mean)):
    	if detection.class_to_label(tab_labels[i]) == boisson:
    		if indice == -1:
    			indice=i
	    	elif tab_mean[indice][4] < tab_mean[i][4]:
	    		indice=i
    
    if indice==-1:
    	return None, None, None, None, 0
    
    return float(tab_mean[indice][0]), float(tab_mean[indice][1]), float(tab_mean[indice][2]), float(tab_mean[indice][3]), float(tab_mean[indice][4])


def recup_bottle(x1, y1, x2, y2, limb, coef_boisson):
	traj_options = TrajectoryOptions()
	traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
    
	waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
	poseStamped = PoseStamped()
	
	endpoint_state = limb.tip_state('right_hand')
	pose = endpoint_state.pose
	
	height = (y2-y1)
	deep = (0.15/height)/coef_boisson
	
	offset = ((x1+x2)/2) - 0.5
	
	offset = offset * 1/height / coef_boisson / 2

	
	print("offset = " + str(offset))
	print("hauteur = " + str(y2-y1))
	print("deep = " + str(deep))
	
	####### CONFIGURE PRE
	rot = PyKDL.Rotation.RPY(0, 0, 0)
	trans = PyKDL.Vector(offset, 0, 0)
	
	f = PyKDL.Frame(rot, trans)
	pose = posemath.toMsg(f*posemath.fromMsg(pose))
	
	poseStamped.pose = pose
	
	joint_angles = limb.joint_ordered_angles()
	waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)
	
	traj.append_waypoint(waypoint.to_msg())
	
	####### CONFIGURE GET
	rot = PyKDL.Rotation.RPY(0, 0, 0)
	trans = PyKDL.Vector(0, deep, 0)
	
	f = PyKDL.Frame(rot, trans)
	pose = posemath.toMsg(f*posemath.fromMsg(pose))
	
	poseStamped.pose = pose
	
	joint_angles = limb.joint_ordered_angles()
	waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)
	
	traj.append_waypoint(waypoint.to_msg())
	
	result = traj.send_trajectory()
	if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

	if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
	else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)



def pour_glass():
    args = parser.parse_args()
    if not args.dropOff or args.dropOff < 1 or args.dropOff > 3:
    	exit(1)
    if not args.drink in coefs_boissons:
    	exit(2)
    init_thread = threading.Thread(target=init_detection)
    init_thread.start()

    rp = intera_interface.RobotParams()

    print("Initializing node... ")
    rospy.init_node('pour_with_detection')
    
    l = intera_interface.Limb('right')
    li = intera_interface.Lights()
    li.set_light_state("head_red_light",True)
    
    cam = 'right_hand_camera'

    l.set_joint_position_speed(0.3)	
    l.move_to_joint_positions(pos_detect_bottle)
    
    cameras = intera_interface.Cameras()
    cameras.start_streaming(cam)
    cameras.set_exposure(cam, 5)
    
    init_thread.join()
    
    li.set_light_state("head_red_light",False)
    li.set_light_state("head_green_light",True)
    
    global detection
    
    cameras.set_callback(cam, show_image_callback,
        rectify_image=True)

    global tab
    
    time.sleep(3)
    
    cameras.stop_streaming(cam)
    
    l.move_to_joint_positions(pos_gen)
    
    t_m, t_l = extract_mean_coord(tab)
    
    x1, y1, x2, y2, conf = coord_from_best(t_m,t_l,args.drink)
    
    print(str(x1) +","+ str(y1)+","+str(x2)+","+str(y2)+","+str(conf))
    
    li.set_light_state("head_green_light",False)
    

    if conf < 0.5:
    	l.move_to_neutral()
    else:
    	recup_bottle(x1, y1, x2, y2, l, coefs_boissons[args.drink])

	
	

if __name__ == '__main__':
	try:
		pour_glass()
	except rospy.ROSInterruptException:
		pass
