#!/usr/bin/env python

import rospy
import intera_interface
import numpy as np
import cv2
import torch
import threading
import time

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

################ POSITIONS ###############

pos_detect_glass = {'right_j0': -0.956154296875, 'right_j1': -0.7500234375, 'right_j2': -0.0840078125, 'right_j3': 2.1074306640625, 'right_j4': 0.52949609375, 'right_j5': -2.9210869140625, 'right_j6': -1.271640625}

pos_gen = {'right_j0': -0.8131884765625, 'right_j1': -0.7814609375, 'right_j2': -0.4102216796875, 'right_j3': 2.03480078125, 'right_j4': 0.441275390625, 'right_j5': -0.8432001953125, 'right_j6': -1.2724794921875}

wpt_opts = MotionWaypointOptions(max_linear_speed=0.6,
                                         max_linear_accel=0.6,
                                         max_rotational_speed=1.57,
                                         max_rotational_accel=1.57,
                                         max_joint_speed_ratio=1.0)

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
    
    detection = DetectionYolov5('/home/loan/sawyer_vision_bartender/model/best_small.pt')

def extract_mean_coord(tab):
    tab_mean=[]
    j = len(tab)-1
    coef=0
    while j > len(tab)/2 :
    	labels, coord = tab[j]
    	n = len(labels)
    	coord = coord.clone()
    	for i in range(n):
    	    if coef == 0:
    	    	tab_mean.append(coord[i])
    	    if n == len(tab_mean):
    	    	for k in range(5):
    	    		tab_mean[i][k]= (tab_mean[i][k] * coef + coord[i][k]) / (coef + 1)
    	j-=1
    	coef+=1
    return tab_mean
    
def coord_from_best(tab):
    if not tab:
    	return None, None, None, None, 0

    indice=0
    
    for i in range(1,len(tab)):
    	if tab[indice][4] < tab[i][4]:
    		indice=i
    
    return float(tab[indice][0]), float(tab[indice][1]), float(tab[indice][2]), float(tab[indice][3]), float(tab[indice][4])
    
def recup_glass(x1, y1, x2, y2, limb,traj):
	pre_point = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
	get_point = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
	got_point = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
	poseStamped = PoseStamped()
	
	endpoint_state = limb.tip_state('right_hand')
	pose = endpoint_state.pose
	
	offset = (0.5 - (x1+x2)/2)/2.4
	rospy.loginfo(offset)
	
	####### CONFIGURE PRE
	rot = PyKDL.Rotation.RPY(0, 0, 0)
	trans = PyKDL.Vector(offset, 0, 0)
	
	f = PyKDL.Frame(rot, trans)
	pose = posemath.toMsg(f*posemath.fromMsg(pose))
	
	poseStamped.pose = pose
	
	joint_angles = limb.joint_ordered_angles()
	pre_point.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)
	
	traj.append_waypoint(pre_point.to_msg())
	#traj.append_waypoint(get_point.to_msg())
	
	result = traj.send_trajectory()
	if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

	if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
	else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
                         
	####### CONFIGURE GET
	endpoint_state = limb.tip_state('right_hand')
	pose = endpoint_state.pose
	rot = PyKDL.Rotation.RPY(0, 0, 0)
	trans = PyKDL.Vector(0, -0.15, 0)
	
	f = PyKDL.Frame(rot, trans)
	pose = posemath.toMsg(f*posemath.fromMsg(pose))
	
	poseStamped = PoseStamped()
	poseStamped.pose = pose
	
	joint_angles = limb.joint_ordered_angles()
	pre_point.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)
	traj.append_waypoint(get_point.to_msg())
	
	result = traj.send_trajectory()
	if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

	if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
	else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

	


def glass_detection():
    
    init_thread = threading.Thread(target=init_detection)
    init_thread.start()

    rp = intera_interface.RobotParams()

    print("Initializing node... ")
    rospy.init_node('glass_detection')
    
    l = intera_interface.Limb('right')
    li = intera_interface.Lights()
    li.set_light_state("head_red_light",True)
    
    traj_options = TrajectoryOptions()
    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
    traj = MotionTrajectory(trajectory_options = traj_options, limb = l)    
    
    cam = 'right_hand_camera'

    l.set_joint_position_speed(0.3)	
    l.move_to_joint_positions(pos_detect_glass)
    
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
    
    time.sleep(2)
    
    cameras.stop_streaming(cam)
    l.move_to_joint_positions(pos_gen)    
    
    x1, y1, x2, y2, conf = coord_from_best(extract_mean_coord(tab))
    
    print(str(x1) +","+ str(y1)+","+str(x2)+","+str(y2)+","+str(conf))
    
    li.set_light_state("head_green_light",False)
    
    if conf < 0.5:
    	l.move_to_neutral()
    else:
    	recup_glass(x1, y1, x2, y2, l, traj)
    
 
    
    
if __name__ == '__main__':
	try:
		glass_detection()
	except rospy.ROSInterruptException:
		pass
'''		
pre_point = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
	
	endpoint_state = limb.tip_state('right_hand')
	pose = endpoint_state.pose
	
	pose.position.x = 0.58 - (x1 * 0.2)
	pose.position.y =  -0.5
	pose.position.z =  -0.38
	
	pose.orientation.x = 0.57 - (x1 * 0.08)
	pose.orientation.y = -0.49 - (x1 * 0.03)
	pose.orientation.z = 0.40 + (x1 * 0.1)
	pose.orientation.w = 0.53 - (x1 * 0.05)
	
	rospy.loginfo(pose)
	
	poseStamped = PoseStamped()
	poseStamped.pose = pose
	
	joint_angles = limb.joint_ordered_angles()
	pre_point.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)
	
	traj.append_waypoint(pre_point.to_msg())
	
	result = traj.send_trajectory()
	if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

	if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
	else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
'''           
