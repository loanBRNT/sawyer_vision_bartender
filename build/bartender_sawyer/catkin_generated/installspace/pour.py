#!/usr/bin/env python3

import rospy
import intera_interface
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

pos_start = [0.0178681640625, 0.001845703125, 1.503634765625, 2.4913583984375, 2.9768037109375, 0.8620634765625, 0.277310546875]

pos_pre_recup_bottle = {"Water":[0.26408984375, 0.1462138671875, 1.4781796875, 2.0948603515625,  2.97659765625, 0.777044921875, 0.2845927734375],"Coca":[1.8395498046875, -0.086404296875,  -0.0936083984375,  1.3503837890625, 2.978443359375, 1.1921962890625, 1.878099609375], "Orange":[0.113380859375,  0.2030322265625,  1.6344296875, 1.5729638671875, 2.9780302734375,  0.12437109375,0.1842099609375]}

pos_recup_bottle = {"Water":[0.2624375, 0.521849609375, 1.7164619140625, 1.709037109375, 2.9274326171875, 0.2864677734375, -0.09963671875],"Coca":[1.871375, 0.203958984375,  -0.252384765625, 0.8260419921875,2.9751513671875, 1.0068076171875, 2.0497763671875], "Orange":[0.2179677734375, 0.2368447265625, 1.6838330078125,  1.40006640625, 2.9587021484375,  0.0920673828125, 0.1842099609375]}

pos_get_bottle = {"Water":[0.3136025390625, -0.5513134765625, 1.6816669921875, 1.9379521484375, 2.97453125, 0.0645283203125, 0.9403134765625],"Coca":[1.956744140625, -0.2618828125, -0.4530830078125, 1.169671875, 2.8605791015625,  0.9363916015625, 2.3160263671875], "Orange":[0.1653623046875, -0.5455634765625,1.674015625,  1.539099609375, 2.97741015625, -0.2992490234375, 0.856263671875]}

pos_interm_coca = [0.9757421875, -0.199314453125, -0.547630859375, 1.1335419921875,  2.9047998046875, 0.96819140625, 2.3736865234375]

pos_go_to_glass = [0.64188671875, -0.369873046875,  -0.3039892578125,  1.710625,  2.9776171875,  1.3243916015625, 1.912365234375]

pos_pre_pour = [{'right_j0': -0.813703125, 'right_j1': 1.1702578125, 'right_j2': 2.9353994140625, 'right_j3': 1.3329501953125, 'right_j4': 1.555283203125, 'right_j5': -1.37959765625, 'right_j6': -0.7599150390625},{'right_j0': -0.57901953125, 'right_j1': 1.1133623046875, 'right_j2': 2.7001943359375, 'right_j3': 1.286443359375, 'right_j4': 1.5670322265625, 'right_j5': -0.9108017578125, 'right_j6': -0.6902607421875},{'right_j0': -0.18113671875, 'right_j1': 1.0266357421875, 'right_j2': 2.8971220703125, 'right_j3': 1.1486650390625, 'right_j4': 1.634646484375, 'right_j5': -0.5609541015625, 'right_j6': -0.7536923828125}]


pos_pour = [{'right_j0': -0.79362890625, 'right_j1': 1.1752998046875, 'right_j2': 2.8832080078125, 'right_j3': 1.322591796875, 'right_j4': 1.4317255859375, 'right_j5': -1.2610888671875, 'right_j6': -1.38653515625},{'right_j0': -0.5678994140625, 'right_j1': 1.123646484375, 'right_j2': 2.6999150390625, 'right_j3': 1.3081806640625, 'right_j4': 1.4762939453125, 'right_j5': -0.898265625, 'right_j6': -1.2859072265625},{'right_j0': -0.109994140625, 'right_j1': 0.9854912109375, 'right_j2': 3.012224609375, 'right_j3': 1.0032978515625, 'right_j4': 1.55776171875, 'right_j5': -0.6307890625, 'right_j6': -1.6247158203125}]


pos_end_pour = [{'right_j0': -0.782919921875, 'right_j1': 1.2921015625, 'right_j2': 3.0203994140625, 'right_j3': 2.112330078125, 'right_j4': 1.25609765625, 'right_j5': -1.495486328125, 'right_j6': 0.8799287109375},{'right_j0': -1.08826953125, 'right_j1': 1.2884990234375, 'right_j2': 2.36146484375, 'right_j3': 2.076525390625, 'right_j4': 1.6325810546875, 'right_j5': -1.0225849609375, 'right_j6': 0.6602490234375},{'right_j0': -0.0627724609375, 'right_j1': 1.2460947265625, 'right_j2': 3.03573046875, 'right_j3': 1.70759375, 'right_j4': 1.2697314453125, 'right_j5': -0.7821826171875, 'right_j6': 0.7451904296875}
]

wpt_opts = MotionWaypointOptions(max_linear_speed=0.6,
                                         max_linear_accel=0.6,
                                         max_rotational_speed=1.57,
                                         max_rotational_accel=1.57,
                                         max_joint_speed_ratio=1.0,
                                         corner_distance=0.1)


parser = argparse.ArgumentParser()
parser.add_argument('-d','--drink', type=str, help='Drink to pour')
parser.add_argument('-do','--dropOff', type=int, help='Position numbers (between 1 and 3) where the glass is')


def pour_glasse_defined():
	args = parser.parse_args()
	
	if (args.drink not in pos_recup_bottle) :
		exit(2)
	
	if (args.dropOff < 1) or (args.dropOff > 3):
		exit(4)
	
	rospy.init_node('pour_glasse_defined')
	limb = intera_interface.Limb('right')
	
	grip = intera_interface.Gripper()
	grip.open()
	
	traj_options = TrajectoryOptions()
	traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
    
	joint_names = limb.joint_names()
      
	waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
	poseStamped = PoseStamped()
	
	waypoint.set_joint_angles(pos_start, 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())

	waypoint.set_joint_angles(pos_pre_recup_bottle[args.drink], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())

	waypoint.set_joint_angles(pos_recup_bottle[args.drink], 'right_hand', joint_names)
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

	grip.close()
	
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
	
	waypoint.set_joint_angles(pos_get_bottle[args.drink], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_interm_coca, 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_go_to_glass, 'right_hand', joint_names)
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
	
	
	
if __name__ == '__main__':
	try:
		pour_glasse_defined()
	except rospy.ROSInterruptException:
		pass
