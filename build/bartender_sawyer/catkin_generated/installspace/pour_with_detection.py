#!/usr/bin/env python3

import rospy
import intera_interface
import time
import argparse

pos_detect_bottle = {'right_j0': 1.482083984375, 'right_j1': 0.0318505859375, 'right_j2': -0.0857490234375, 'right_j3': 1.4007880859375, 'right_j4': -0.0340009765625, 'right_j5': -2.977314453125, 'right_j6': 0.2768974609375}

parser = argparse.ArgumentParser()
parser.add_argument('-d','--drink', type=str, help='Drink to pour')
parser.add_argument('-do','--dropOff', type=int, help='Position numbers (between 1 and 3) where the glass is')


def pour_glasse_defined():
	args = parser.parse_args()
	
	rospy.init_node('pour_glasse_detection')
	limb = intera_interface.Limb('right')
	
	grip = intera_interface.Gripper()
	grip.open()

	limb.set_joint_position_speed(0.3)
	limb.move_to_joint_positions(pos_detect_bottle)
	
	

if __name__ == '__main__':
	try:
		pour_glasse_defined()
	except rospy.ROSInterruptException:
		pass
