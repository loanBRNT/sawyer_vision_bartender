#!/usr/bin/env python3

import rospy
import intera_interface
import time
import argparse

pos_detect_bottle = {'right_j0': 0.6166103515625, 'right_j1': 0.30004296875, 'right_j2': 1.7991611328125, 'right_j3': 1.4206669921875, 'right_j4': 2.6511640625, 'right_j5': -1.3242490234375, 'right_j6': 0.4215390625}


pos_pre_recup_bottle = {"Water":[{'right_j0': 0.3017421875, 'right_j1': 0.450302734375, 'right_j2': 1.7270009765625, 'right_j3': 1.81893359375, 'right_j4': 2.8303291015625, 'right_j5': 0.3609765625, 'right_j6': -0.0738671875}],"Coca":[{'right_j0': 0.6179482421875, 'right_j1': 0.4054921875, 'right_j2': 1.7760791015625, 'right_j3': 1.838830078125, 'right_j4': 2.9782373046875, 'right_j5': 0.680109375, 'right_j6': -0.0740859375}], "Orange":[{'right_j0': 0.2600634765625, 'right_j1': 0.04612890625, 'right_j2': 1.5538056640625, 'right_j3': 1.74796484375, 'right_j4': 2.9770107421875, 'right_j5': 0.272265625, 'right_j6': 0.2288818359375}]}

pos_recup_bottle = {"Water":[{'right_j0': 0.5046708984375, 'right_j1': 0.3083662109375, 'right_j2': 1.7453271484375, 'right_j3': 1.4765576171875, 'right_j4': 2.9782373046875, 'right_j5': 0.1718828125, 'right_j6': -0.0846728515625}],"Coca":[{'right_j0': 0.817076171875, 'right_j1': 0.368078125, 'right_j2': 1.8805166015625, 'right_j3': 1.4895244140625, 'right_j4': 2.9782373046875, 'right_j5': 0.6469150390625, 'right_j6': -0.07429296875}], "Orange":[{'right_j0': 0.4813564453125, 'right_j1': 0.04849609375, 'right_j2': 1.5695517578125, 'right_j3': 1.413177734375, 'right_j4': 2.9770107421875, 'right_j5': 0.2805029296875, 'right_j6': 0.2288818359375}]}

pos_get_bottle = {"Water":[{'right_j0': 0.4388486328125, 'right_j1': -0.4932666015625, 'right_j2': 1.7197822265625, 'right_j3': 1.7293671875, 'right_j4': 2.9527373046875, 'right_j5': -0.0201787109375, 'right_j6': 0.930359375}],"Coca":[{'right_j0': 0.54326171875, 'right_j1': -0.54596875, 'right_j2': 1.7954169921875, 'right_j3': 2.0066474609375, 'right_j4': 2.975564453125, 'right_j5': 0.1237509765625, 'right_j6': 0.8348828125}], "Orange":[{'right_j0': 0.4308017578125, 'right_j1': -0.6693623046875, 'right_j2': 1.5098525390625, 'right_j3': 1.6396923828125, 'right_j4': 2.9212607421875, 'right_j5': 0.0519794921875, 'right_j6': 1.1033525390625}]}


pos_go_to_glass = {'right_j0': -0.758517578125, 'right_j1': -0.2010634765625, 'right_j2': 2.0430732421875, 'right_j3': 1.2065234375, 'right_j4': 2.524662109375, 'right_j5': -0.8031376953125, 'right_j6': 0.9324248046875}


pos_pre_pour = [{'right_j0': -0.813703125, 'right_j1': 1.1702578125, 'right_j2': 2.9353994140625, 'right_j3': 1.3329501953125, 'right_j4': 1.555283203125, 'right_j5': -1.37959765625, 'right_j6': -0.7599150390625},{'right_j0': -0.57901953125, 'right_j1': 1.1133623046875, 'right_j2': 2.7001943359375, 'right_j3': 1.286443359375, 'right_j4': 1.5670322265625, 'right_j5': -0.9108017578125, 'right_j6': -0.6902607421875},{'right_j0': -0.18113671875, 'right_j1': 1.0266357421875, 'right_j2': 2.8971220703125, 'right_j3': 1.1486650390625, 'right_j4': 1.634646484375, 'right_j5': -0.5609541015625, 'right_j6': -0.7536923828125}]


pos_pour = [{'right_j0': -0.79362890625, 'right_j1': 1.1752998046875, 'right_j2': 2.8832080078125, 'right_j3': 1.322591796875, 'right_j4': 1.4317255859375, 'right_j5': -1.2610888671875, 'right_j6': -1.38653515625},{'right_j0': -0.5678994140625, 'right_j1': 1.123646484375, 'right_j2': 2.6999150390625, 'right_j3': 1.3081806640625, 'right_j4': 1.4762939453125, 'right_j5': -0.898265625, 'right_j6': -1.2859072265625},{'right_j0': -0.109994140625, 'right_j1': 0.9854912109375, 'right_j2': 3.012224609375, 'right_j3': 1.0032978515625, 'right_j4': 1.55776171875, 'right_j5': -0.6307890625, 'right_j6': -1.6247158203125}]


pos_end_pour = [{'right_j0': -0.782919921875, 'right_j1': 1.2921015625, 'right_j2': 3.0203994140625, 'right_j3': 2.112330078125, 'right_j4': 1.25609765625, 'right_j5': -1.495486328125, 'right_j6': 0.8799287109375},{'right_j0': -1.08826953125, 'right_j1': 1.2884990234375, 'right_j2': 2.36146484375, 'right_j3': 2.076525390625, 'right_j4': 1.6325810546875, 'right_j5': -1.0225849609375, 'right_j6': 0.6602490234375},{'right_j0': -0.0627724609375, 'right_j1': 1.2460947265625, 'right_j2': 3.03573046875, 'right_j3': 1.70759375, 'right_j4': 1.2697314453125, 'right_j5': -0.7821826171875, 'right_j6': 0.7451904296875}
]

parser = argparse.ArgumentParser()
parser.add_argument('-d','--drink', type=str, help='Drink to pour')
parser.add_argument('-pu','--pickUp', type=int, help='Position numbers (1 or 2) where the drink is to be picked up')
parser.add_argument('-do','--dropOff', type=int, help='Position numbers (between 1 and 3) where the glass is')


def pour_glasse_defined():
	args = parser.parse_args()
	
	if (args.drink not in pos_recup_bottle) :
		exit(2)
	
	if (args.pickUp != 1):
		exit(3)
	
	if (args.dropOff < 1) or (args.dropOff > 3):
		exit(4)
	
	rospy.init_node('pour_glasse_defined')
	limb = intera_interface.Limb('right')
	
	grip = intera_interface.Gripper()
	grip.calibrate()
	grip.open()

	limb.set_joint_position_speed(0.3)
	limb.move_to_joint_positions(pos_detect_bottle)
	print("Detection of bottles")
	time.sleep(2)
	print("Bottle detect")
	limb.move_to_joint_positions(pos_pre_recup_bottle[args.drink][args.pickUp-1])
	limb.set_joint_position_speed(0.1)
	time.sleep(1)
	limb.move_to_joint_positions(pos_recup_bottle[args.drink][args.pickUp-1])
	grip.close()
	time.sleep(2)
	limb.move_to_joint_positions(pos_get_bottle[args.drink][args.pickUp-1])
	time.sleep(1)
	limb.move_to_joint_positions(pos_go_to_glass)
	time.sleep(1)
	limb.set_joint_position_speed(0.2)
	limb.move_to_joint_positions(pos_pre_pour[args.dropOff-1])
	time.sleep(1)
	limb.set_joint_position_speed(0.1)
	limb.move_to_joint_positions(pos_pour[args.dropOff-1])
	time.sleep(2)
	limb.move_to_joint_positions(pos_pre_pour[args.dropOff-1])
	time.sleep(1)
	limb.move_to_joint_positions(pos_end_pour[args.dropOff-1])
	time.sleep(1)
	limb.set_joint_position_speed(0.2)
	limb.move_to_joint_positions(pos_get_bottle[args.drink][args.pickUp-1])
	time.sleep(1)
	limb.move_to_joint_positions(pos_recup_bottle[args.drink][args.pickUp-1])
	grip.open()
	time.sleep(2)
	limb.move_to_joint_positions(pos_get_bottle[args.drink][args.pickUp-1])
	limb.move_to_neutral()

if __name__ == '__main__':
	try:
		pour_glasse_defined()
	except rospy.ROSInterruptException:
		pass
