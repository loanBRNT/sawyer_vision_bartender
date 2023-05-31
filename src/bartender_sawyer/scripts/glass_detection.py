#!/usr/bin/env python

import rospy
import intera_interface
import argparse
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

def show_image_callback(img_data, xxx_todo_changeme):
    """The callback function to show image by using CvBridge and cv
    """
    (edge_detection, window_name) = xxx_todo_changeme
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError as err:
        rospy.logerr(err)
        return
    if edge_detection == True:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        # customize the second and the third argument, minVal and maxVal
        # in function cv2.Canny if needed
        get_edge = cv2.Canny(blurred, 5, 100)
        cv_image = np.hstack([get_edge])
    print(cv_image)



def glass_detection():

    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
            " parameters on this robot. Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-c', '--camera', type=str, default="head_camera",
        choices=valid_cameras, help='Setup Camera Name for Camera Display')
    parser.add_argument(
        '-r', '--raw', action='store_true',
        help='Specify use of the raw image (unrectified) topic')
    parser.add_argument(
        '-e', '--edge', action='store_true',
        help='Streaming the Canny edge detection image')
    parser.add_argument(
        '-g', '--gain', type=int,
        help='Set gain for camera (-1 = auto)')
    parser.add_argument(
        '-x', '--exposure', type=float,
        help='Set exposure for camera (-1 = auto)')
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node('glass_detection')
    cameras = intera_interface.Cameras()
    if not cameras.verify_camera_exists(args.camera):
        rospy.logerr("Could not detect the specified camera, exiting.")
        return
    rospy.loginfo("Opening camera '{0}'...".format(args.camera))
    cameras.start_streaming(args.camera)
    rectify_image = not args.raw
    use_canny_edge = args.edge
    cameras.set_callback(args.camera, show_image_callback,
        rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))

    # optionally set gain and exposure parameters
    if args.gain is not None:
        if cameras.set_gain(args.camera, args.gain):
            rospy.loginfo("Gain set to: {0}".format(cameras.get_gain(args.camera)))

    if args.exposure is not None:
        if cameras.set_exposure(args.camera, args.exposure):
            rospy.loginfo("Exposure set to: {0}".format(cameras.get_exposure(args.camera)))

    def clean_shutdown():
        print("Shutting down")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera running. Ctrl-c to quit")
    rospy.spin()

if __name__ == '__main__':
	try:
		glass_detection()
	except rospy.ROSInterruptException:
		pass
