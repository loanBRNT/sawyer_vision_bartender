#!/usr/bin/env python

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

pos_pre_recup_bottle = {"Water":[0.26408984375, 0.1462138671875, 1.4781796875, 2.0948603515625,  2.97659765625, 0.777044921875, 0.2845927734375],"Coca":[1.8395498046875, -0.086404296875,  -0.0936083984375,  1.3503837890625, 2.978443359375, 1.1921962890625, 1.878099609375], "Orange":[-0.17433203125, 0.243109375, 1.523296875, 1.926267578125, 2.909744140625, 0.11697265625, 0.20645703125]}

pos_recup_bottle = {"Water":[0.2624375, 0.521849609375, 1.7164619140625, 1.709037109375, 2.9274326171875, 0.2864677734375, -0.09963671875],"Coca":[1.871375, 0.203958984375,  -0.252384765625, 0.8260419921875,2.9751513671875, 1.0068076171875, 2.0497763671875], "Orange":[0.1790810546875, 0.2911025390625, 1.715451171875, 1.3797548828125, 2.72172265625, -0.199189453125, 0.2904033203125]}

pos_get_bottle = {"Water":[0.3136025390625, -0.5513134765625, 1.6816669921875, 1.9379521484375, 2.97453125, 0.0645283203125, 0.9403134765625],"Coca":[1.956744140625, -0.2618828125, -0.4530830078125, 1.169671875, 2.8605791015625,  0.9363916015625, 2.3160263671875], "Orange":[0.1653623046875, -0.5455634765625,1.674015625,  1.539099609375, 2.97741015625, -0.2992490234375, 0.856263671875]}

pos_interm = [0.9757421875, -0.199314453125, -0.547630859375, 1.1335419921875,  2.9047998046875, 0.96819140625, 2.3736865234375]

pos_go_to_glass = [0.64188671875, -0.369873046875,  -0.3039892578125,  1.710625,  2.9776171875,  1.3243916015625, 1.912365234375]

pos_pre_pour = [[0.1922861328125, 0.035548828125, -0.30747265625, 1.12749609375, 2.97741015625,  1.11075390625, 2.034103515625],[0.16102734375, 0.0102783203125, 0.4033466796875, 1.3783203125, 2.3444384765625, 0.9012734375, 2.659044921875],[0.8948623046875, 0.236021484375, -0.26154296875, 0.9559609375, 2.9749443359375, 1.057625, 1.96664453125]]

pos_pour = [[-0.033791015625, -0.0165419921875, 0.156546875, 1.2946455078125, 2.771919921875, 0.89508984375, 1.3623994140625],[0.156486328125, -0.040796875, 0.33340625, 1.421677734375,  2.2584384765625, 0.9436083984375, 3.4513115234375],[0.843587890625, 0.4864814453125, -0.4678994140625, 0.732396484375, 2.9770107421875, 1.0247802734375, 2.6678759765625]]

pos_pouring= [[0.1152265625, 0.074287109375, -0.04066796875, 1.0553896484375, 2.90252734375, 0.586517578125, 0.0468515625],[0.156486328125, -0.040796875, 0.33340625, 1.421677734375,  2.2584384765625, 0.9436083984375, 3.4513115234375],[0.84534375, 0.444861328125, -0.3554404296875, 0.7752666015625, 2.9757705078125, 1.02932421875, 3.857361328125]]

pos_neutre = [0.0018916015625, -1.178173828125, -0.0025615234375,2.1771728515625,  0.0064345703125,0.5698369140625, 3.31188671875]

wpt_opts = MotionWaypointOptions(max_linear_speed=0.6,
                                         max_linear_accel=0.6,
                                         max_rotational_speed=1.57,
                                         max_rotational_accel=1.57,
                                         max_joint_speed_ratio=0.9,
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

	if not result.result:
            rospy.logerr('ERROR %s',11)
            return

	grip.close()
	
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
	
	waypoint.set_joint_angles(pos_get_bottle[args.drink], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_interm, 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_go_to_glass, 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_pre_pour[args.dropOff-1], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_pour[args.dropOff-1], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_pouring[args.dropOff-1], 'right_hand', joint_names)
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
            return
	#'''
	time.sleep(1)
	
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
	
	waypoint.set_joint_angles(pos_pour[args.dropOff-1], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_pre_pour[args.dropOff-1], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_go_to_glass, 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_interm, 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_get_bottle[args.drink], 'right_hand', joint_names)
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
            return
        
	grip.open()
	
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
	
	waypoint.set_joint_angles(pos_get_bottle[args.drink], 'right_hand', joint_names)
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
            return
	#'''
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
	
	waypoint.set_joint_angles(pos_neutre, 'right_hand', joint_names)
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
            return
	
	exit(0)
	
if __name__ == '__main__':
	try:
		pour_glasse_defined()
	except rospy.ROSInterruptException:
		pass
