#!/usr/bin/env python

import rospy
import intera_interface
import time
import argparse

################ POSITIONS ###############

pos_detect_glass = {'right_j0': -0.80207421875, 'right_j1': 1.252166015625, 'right_j2': -1.4084111328125, 'right_j3': 0.58323046875, 'right_j4': 2.0536455078125, 'right_j5': -2.509525390625, 'right_j6': -1.291484375}

pos_pre_recup_glass = [ {'right_j0': -0.532666015625, 'right_j1': 1.27151953125, 'right_j2': -1.439171875, 'right_j3': 0.9640185546875, 'right_j4': 1.8421640625, 'right_j5': -0.68358203125, 'right_j6': -0.6645400390625}, {'right_j0': -0.624029296875, 'right_j1': 1.2078125, 'right_j2': -1.577265625, 'right_j3': 0.831654296875, 'right_j4': 1.791888671875, 'right_j5': -0.6365478515625, 'right_j6': -0.6614287109375}, {'right_j0': -0.895431640625, 'right_j1': 1.2117236328125, 'right_j2': -1.7453271484375, 'right_j3': 0.28679296875, 'right_j4': 2.1689140625, 'right_j5': -0.942033203125, 'right_j6': -1.3567734375}]

pos_recup_glass = [{'right_j0': -1.0360048828125, 'right_j1': 1.1023505859375, 'right_j2': -1.647775390625, 'right_j3': 0.676875, 'right_j4': 1.33, 'right_j5': -0.6297568359375, 'right_j6': -0.534462890625},{'right_j0': -1.014908203125, 'right_j1': 1.0770478515625, 'right_j2': -1.7496494140625, 'right_j3': 0.5284404296875, 'right_j4': 1.60949609375, 'right_j5': -0.4598740234375, 'right_j6': -0.775044921875},{'right_j0': -1.0514345703125, 'right_j1': 1.0543115234375, 'right_j2': -1.7487919921875, 'right_j3': 0.207107421875, 'right_j4': 1.9799365234375, 'right_j5': -0.4508369140625, 'right_j6': -1.3588388671875}]

pos_post_recup_glass = [{'right_j0': -0.9970087890625, 'right_j1': 0.794609375, 'right_j2': -1.80078515625, 'right_j3': 1.1840732421875, 'right_j4': 1.5194033203125, 'right_j5': -0.7608154296875, 'right_j6': -0.534255859375},{'right_j0': -0.910771484375, 'right_j1': 0.8328583984375, 'right_j2': -1.834767578125, 'right_j3': 0.9932275390625, 'right_j4': 1.782619140625, 'right_j5': -0.7408935546875, 'right_j6': -0.7756650390625},{'right_j0': -1.0290068359375, 'right_j1': 0.61950390625, 'right_j2': -1.74821484375, 'right_j3': 0.3257744140625, 'right_j4': 2.148541015625, 'right_j5': -0.92559765625, 'right_j6': -1.659173828125}]

pos_pre_put_glass = [{'right_j0': 0.305041015625, 'right_j1': 0.7668427734375, 'right_j2': -1.3863125, 'right_j3': 0.9546611328125, 'right_j4': 0.92139453125, 'right_j5': -0.9726455078125, 'right_j6': -0.5043798828125},{'right_j0': 0.3453818359375, 'right_j1': 0.9683125, 'right_j2': -2.0013759765625, 'right_j3': 0.81813671875, 'right_j4': 1.3921142578125, 'right_j5': -0.627083984375, 'right_j6': -0.389177734375},{'right_j0': 0.387470703125, 'right_j1': 0.8612421875, 'right_j2': -1.4693369140625, 'right_j3': 0.057345703125, 'right_j4': 1.957291015625, 'right_j5': -0.8586162109375, 'right_j6': -1.7408095703125}]

pos_put_glass = [{'right_j0': 0.2506796875, 'right_j1': 0.863505859375, 'right_j2': -1.3675439453125, 'right_j3': 0.7778564453125, 'right_j4': 0.798353515625, 'right_j5': -0.8021044921875, 'right_j6': -0.5045869140625},{'right_j0': 0.2697568359375, 'right_j1': 0.911126953125, 'right_j2': -2.0144599609375, 'right_j3': 0.5911435546875, 'right_j4': 0.9641943359375, 'right_j5': -0.46542578125, 'right_j6': -0.217849609375},{'right_j0': 0.0537646484375, 'right_j1': 0.978390625, 'right_j2': -1.7764404296875, 'right_j3': -0.5532861328125, 'right_j4': 2.7120517578125, 'right_j5': -0.6055224609375, 'right_j6': -2.3341572265625}]

pos_post_put_glass = [{'right_j0': 0.464845703125, 'right_j1': 0.8338876953125, 'right_j2': -1.0946845703125, 'right_j3': 1.028482421875, 'right_j4': 0.739169921875, 'right_j5': -0.9272373046875, 'right_j6': -0.501888671875},{'right_j0': 0.7132861328125, 'right_j1': 0.90948046875, 'right_j2': -1.47756640625, 'right_j3': 0.9297744140625, 'right_j4': 0.7937958984375, 'right_j5': -0.53548046875, 'right_j6': -0.1802529296875},{'right_j0': 0.0200361328125, 'right_j1': 1.2547392578125, 'right_j2': -1.7145029296875, 'right_j3': -0.967587890625, 'right_j4': 2.2528876953125, 'right_j5': -1.5471689453125, 'right_j6': -2.3343642578125}]

pos_end = {'right_j0': 0.9390166015625, 'right_j1': 0.5649169921875, 'right_j2': -1.550005859375, 'right_j3': 1.367509765625, 'right_j4': 0.678345703125, 'right_j5': -0.4288623046875, 'right_j6': -0.1827451171875}

############################################

parser = argparse.ArgumentParser()
parser.add_argument('-pu','--pickUp', type=int, help='Position numbers (between 1 and 3) where the glass is to be picked up')
parser.add_argument('-do','--dropOff', type=int, help='Position numbers (between 1 and 3) where the glass is to be droped off')

def get_glass():
	args = parser.parse_args()
	
	if (args.pickUp < 1) or (args.pickUp > 3):
		exit(2)
	
	if (args.dropOff < 1) or (args.dropOff > 3):
		exit(3)

	rospy.init_node('pour_glasse_defined')
	limb = intera_interface.Limb('right')
	grip = intera_interface.Gripper()
	grip.open()
	
	print("start")
	limb.set_joint_position_speed(0.3)
	
	limb.move_to_joint_positions(pos_detect_glass)
	time.sleep(1)
	
	limb.set_joint_position_speed(0.2)
	
	limb.move_to_joint_positions(pos_pre_recup_glass[args.pickUp-1])
	time.sleep(1)
	
	limb.move_to_joint_positions(pos_recup_glass[args.pickUp-1])

	time.sleep(2)
	limb.move_to_joint_positions(pos_post_recup_glass[args.pickUp-1])
	limb.set_joint_position_speed(0.1)
	time.sleep(1)
	limb.move_to_joint_positions(pos_end)
	time.sleep(1)
	limb.move_to_joint_positions(pos_pre_put_glass[args.dropOff-1])
	time.sleep(1)
	limb.move_to_joint_positions(pos_put_glass[args.dropOff-1])

	time.sleep(2)
	limb.move_to_joint_positions(pos_post_put_glass[args.dropOff-1])
	limb.set_joint_position_speed(0.3)
	time.sleep(1)
	limb.move_to_joint_positions(pos_end)
	time.sleep(1)
	limb.move_to_neutral()
	
	
if __name__ == '__main__':
	try:
		get_glass()
	except rospy.ROSInterruptException:
		pass
