#!/usr/bin/env python

import rospy
import intera_interface
import argparse

parser = argparse.ArgumentParser()

rospy.init_node('lights')

led = ['head_blue_light', 'head_green_light', 'head_red_light', 'right_hand_blue_light', 'right_hand_green_light', 'right_hand_red_light']

def lights():
	
	args = parser.parse_args()
	
	Lights = intera_interface.Lights()
	
	
	Lights.set_light_state(led[0],False)
	Lights.set_light_state(led[4],False)

if __name__ == '__main__':
	try:
		lights()
	except rospy.ROSInterruptException:
		pass
