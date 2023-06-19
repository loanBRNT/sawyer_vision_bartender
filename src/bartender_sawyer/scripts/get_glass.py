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

################ POSITIONS ###############

pos_pre_recup_glass = [[-1.6035390625, -0.828837890625, 0.1747294921875,  2.11088671875, 0.052681640625,  -1.1140712890625,  4.708591796875], [-1.2092255859375,-0.8233896484375, -0.0943388671875,  1.9935185546875, 0.2371923828125, -0.87115234375, 4.7135498046875],[-0.9337197265625, -0.634431640625,  -0.0898271484375, 1.8929345703125, 0.5247314453125, -1.2129697265625, 4.7133427734375]]

pos_recup_glass = [[-1.584224609375,-0.7739677734375,0.1607607421875, 1.8298876953125, 0.0931064453125, -0.756916015625, 4.71313671875],[-1.4670693359375, -0.70964453125, 0.2772626953125, 1.8358076171875, 0.314283203125, -0.9724384765625, 4.567939453125],[-0.987849609375, -0.49490625,  -0.171841796875, 1.505234375, 0.5272236328125, -0.837455078125, 4.71313671875]]

pos_post_recup_glass = [[-1.5826806640625, -0.986171875,0.1585771484375,  1.4592783203125, 0.1536455078125, -0.5032294921875, 4.561548828125],[-1.500373046875, -0.9680859375,  0.2665068359375, 1.360462890625, 0.3930146484375, -0.46851171875, 4.4739609375],[-0.997111328125, -0.4422041015625, -0.710783203125, 0.7013291015625, 1.0053291015625,-0.1591142578125, 4.493740234375]]

pos_pre_bar = [-0.3886259765625, -1.049486328125, 0.494654296875, 1.63075,  0.289830078125,  -0.6344814453125,4.2997275390625]

pos_bar = [0.329591796875, -0.4064501953125, -0.088365234375, 1.908662109375, 0.3708466796875, -1.2654140625, -1.3499560546875]

pos_pre_put_glass = [[0.305041015625, 0.7668427734375, -1.3863125, 0.9546611328125, 0.92139453125, -0.9726455078125, -0.5043798828125],[0.3453818359375, 0.9683125, -2.0013759765625,  0.81813671875, 1.3921142578125,-0.627083984375, -0.389177734375],[ 0.524583984375, 0.830498046875, -0.901619140625, 0.2698203125, 1.2100576171875, -0.9083359375, -1.3435654296875]]

pos_put_glass = [[ 0.2506796875,0.863505859375, -1.3675439453125, 0.7778564453125, 0.798353515625,-0.8021044921875,-0.5045869140625],[0.2697568359375, 0.911126953125,-2.0144599609375,  0.5911435546875, 0.9641943359375, -0.46542578125, -0.217849609375],[0.5198369140625, 0.842326171875, -0.901908203125, 0.2603271484375, 1.0972802734375, -0.60531640625, -1.3435654296875]]

pos_post_put_glass = [[ 0.464845703125,  0.8338876953125, -1.0946845703125, 1.028482421875, 0.739169921875, -0.9272373046875, -0.501888671875],[0.751046875, 0.8965205078125,-1.447544921875, 0.9766787109375, 0.8419921875, -0.55315625,-0.1881396484375],[1.15976171875, 0.81074609375, -0.8974140625, 0.9020009765625, 1.5620869140625, -0.693859375, -0.9985849609375]]

pos_end = [0.9390166015625, 0.5649169921875, -1.550005859375, 1.367509765625, 0.678345703125, -0.4288623046875, -0.1827451171875]

pos_inter = [0.0444833984375, -0.450109375, 0.219087890625, 2.1285087890625, 0.058478515625, -1.8043427734375, 3.3572685546875]

pos_neutre= [0.0018916015625, -1.178173828125, -0.0025615234375,2.1771728515625,  0.0064345703125,0.5698369140625, 3.31188671875]

wpt_opts = MotionWaypointOptions(max_linear_speed=0.6,
                                         max_linear_accel=0.6,
                                         max_rotational_speed=1.57,
                                         max_rotational_accel=1.57,
                                         max_joint_speed_ratio=0.9,
                                         corner_distance=0.1)

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
	
	traj_options = TrajectoryOptions()
	traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
    
	joint_names = limb.joint_names()
      
	waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
	poseStamped = PoseStamped()  

	waypoint.set_joint_angles(pos_pre_recup_glass[args.pickUp-1], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_recup_glass[args.pickUp-1], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())

	waypoint.set_joint_angles(pos_post_recup_glass[args.pickUp-1], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	
	waypoint.set_joint_angles(pos_pre_bar, 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_bar, 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_pre_put_glass[args.dropOff-1], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_put_glass[args.dropOff-1], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_post_put_glass[args.dropOff-1], 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_end, 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
	waypoint.set_joint_angles(pos_inter, 'right_hand', joint_names)
	traj.append_waypoint(waypoint.to_msg())
	
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

	
	
if __name__ == '__main__':
	try:
		get_glass()
	except rospy.ROSInterruptException:
		pass
