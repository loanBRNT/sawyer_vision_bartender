#!/usr/bin/env python

import rospy
import intera_interface

pos1 = {'right_j0': -0.0461650390625, 'right_j1': -0.4399404296875, 'right_j2': -0.8729248046875, 'right_j3': 0.57158984375, 'right_j4': 2.9741181640625, 'right_j5': -1.6215107421875, 'right_j6': 1.568328125}
pos2 = {'right_j0': 0.0188974609375, 'right_j1': -0.2028134765625, 'right_j2': -1.3135029296875, 'right_j3': -0.0869921875, 'right_j4': 1.4694775390625, 'right_j5': -1.6114140625, 'right_j6': 1.5714267578125}

def hi():
	rospy.init_node('hello')
	limb = intera_interface.Limb('right')
	print("Initialisation")
	limb.move_to_neutral()
	for e in range(3):
		limb.move_to_joint_positions(pos1)
		limb.move_to_joint_positions(pos2)
	print("Fin d'execution")
	limb.move_to_neutral()
	
if __name__ == '__main__':
	try:
		hi()
	except rospy.ROSInterruptException:
		pass
