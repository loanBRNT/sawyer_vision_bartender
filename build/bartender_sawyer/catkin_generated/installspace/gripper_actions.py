#!/usr/bin/env python3

import rospy
import intera_interface
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-a','--action', type=str, help='Close=c')

rospy.init_node('gripper_actions')
grip = intera_interface.Gripper()

def grip_actions():
	
	args = parser.parse_args()
	
	if not grip.is_ready():
		print("erreur")
		exit(1)
	
	if args.action=="c":
		grip.close()
	else:
		grip.open()

if __name__ == '__main__':
	try:
		grip_actions()
	except rospy.ROSInterruptException:
		pass
