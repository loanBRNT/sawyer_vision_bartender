#!/usr/bin/env python

import rospy
import intera_interface

def initialisation():
	rospy.init_node('init')
	
	HeadDisplay = intera_interface.HeadDisplay()
	RobotEnable = intera_interface.RobotEnable()
	Gripper = intera_interface.Gripper()
	Limb = intera_interface.Limb()
	
	RobotEnable.enable()
	
	Gripper.calibrate()
	Gripper.open()
	
	Limb.move_to_neutral()
	
	HeadDisplay.display_image('background.png')
	
if __name__ == '__main__':
	try:
		initialisation()
	except rospy.ROSInterruptException:
		pass
