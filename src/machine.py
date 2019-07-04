#!/usr/bin/env python

# Copyright (c) 2018 NVIDIA Corporation. All rights reserved.
# This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
# https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

"""
This file starts a ROS node to run DOPE,
listening to an image topic and publishing poses.
"""

from __future__ import print_function
import yaml
import sys

import numpy as np
import cv2
import tf
import math
import rospy
import rospkg
import rosparam
from geometry_msgs.msg import PoseStamped
from time import sleep

import roslaunch
# Import DOPE code
rospack = rospkg.RosPack()
g_path2package = rospack.get_path('dope')
# planner_path = rospack.get_path('cruzr_planner')
planner_path = rospack.get_path('walker_planner')


def start_octomap_server():
    # package = 'cruzr_planner'
    # executable = '.launch'
    # node = roslaunch.core.Node(package, executable)
    #
    launch = roslaunch.scriptapi.ROSLaunch()
    # launch.start()

    # process = launch.launch(node)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [planner_path+"/launch/load_pointcloud.launch"])
    launch.start()
    rospy.loginfo("started")

def wait_till_done_and_publish(param, done_msg, rate, pub, msg):

    while not rospy.is_shutdown():
        try:
            pub.publish(msg)
            if (rospy.get_param(param) == 1):
                rospy.logwarn (done_msg)
                break
            rate.sleep()
        except KeyboardInterrupt:
            print('interrupted!')

def wait_till_done(param, done_msg, rate):

    while not rospy.is_shutdown():
        try:
            if (rospy.get_param(param) == 1):
                rospy.logwarn (done_msg)
                break
            rate.sleep()
        except KeyboardInterrupt:
            print('interrupted!')

def setup_variables():
    rospy.set_param("object_recognition_request", 0)
    rospy.set_param("object_recognition_done", 0)
    rospy.set_param("table_segmentation_request", 0)
    rospy.set_param("table_segmentation_done", 0)
    rospy.set_param("grasp_request", 0)
    rospy.set_param("grasp_done", 0)
    rospy.set_param("walker_planner_request", 0)
    rospy.set_param("walker_planner_done", 0)
    rospy.set_param("controller_request", 0)
    rospy.set_param("controller_done", 0)

if __name__ == "__main__":
    '''Main routine to run DOPE'''


    try:
        rospy.init_node('cruiser_state_machine', anonymous=True, log_level=rospy.INFO)
        rate = rospy.Rate(5)
        setup_variables()
        # sleep(10)

        myargv = rospy.myargv(argv=sys.argv)
        print(myargv)

        # start_octomap_server()

        if '--object_only' not in myargv:
            grasp_publisher = rospy.Publisher(
                '/Grasps',
                PoseStamped,
                queue_size=10
            )
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()

            if '--target' in myargv:
                # Table location
                print("Moving to table location")
                # msg.pose.position.x = -1.1427744124
                # msg.pose.position.y = 0.967387976377
                # msg.pose.position.z = 0.0
                # msg.pose.orientation.x = 0
                # msg.pose.orientation.y = 0
                # msg.pose.orientation.z = -0.633624393243
                # msg.pose.orientation.w = 0.77364082641

                # Hector Slam
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (-0.864, -0.002, 1.06)
                msg.pose.orientation.x = 0
                msg.pose.orientation.y = 0
                msg.pose.orientation.z, msg.pose.orientation.w = (0.998, -0.062)

            elif '--initial' in myargv:
                    # position:
                    #   x: 0.64427947998
                    #   y: 0.363460540771
                    #   z: 0.0
                    # orientation:
                    #   x: 0.0
                    #   y: 0.0
                    #   z: -0.81848372534
                    #   w: 0.574529713203

                # Initial location
                # print("Moving to initial location")
                # msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (-2.615, 1.152, 0.000)
                # msg.pose.orientation.x = 0
                # msg.pose.orientation.y = 0
                # msg.pose.orientation.z, msg.pose.orientation.w = (0.994, -0.113)

                # msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (-3.978, 0.420, 0.000)
                # msg.pose.orientation.x = 0
                # msg.pose.orientation.y = 0
                # msg.pose.orientation.z, msg.pose.orientation.w = (0.879, -0.476)
                #
                # msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (-5.839, -1.054, 0.000)
                # msg.pose.orientation.x = 0
                # msg.pose.orientation.y = 0
                # msg.pose.orientation.z, msg.pose.orientation.w = (1.000, 0.010)

                # Hector Slam
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (1.0, -0.161, 0.8)
                msg.pose.orientation.x = 0
                msg.pose.orientation.y = 0
                msg.pose.orientation.z, msg.pose.orientation.w = (0.0, 1)

            else:
                # Table location
                print("Moving to table location")
                # msg.pose.position.x = -1.1427744124
                # msg.pose.position.y = 0.967387976377
                # msg.pose.position.z = 0.0
                # msg.pose.orientation.x = 0
                # msg.pose.orientation.y = 0
                # msg.pose.orientation.z = -0.633624393243
                # msg.pose.orientation.w = 0.77364082641

                # Hector Slam
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (-0.543, -0.757, 1.06)
                msg.pose.orientation.x = 0
                msg.pose.orientation.y = 0
                msg.pose.orientation.z, msg.pose.orientation.w = (0.984, -0.177)

            rospy.logwarn ("Requesting Initial Planner")
            rospy.set_param("walker_planner_mode", "BASE")
            rospy.set_param("walker_planner_request", 1)

# rostopic pub syscommand std_msgs/String "reload 0.6444 0.36346 -2.108"
            wait_till_done_and_publish("walker_planner_done", "Done Init Planner", rate, grasp_publisher, msg)
            rospy.set_param("walker_planner_done", 0)

            rospy.logwarn ("Requesting Init Controller")
            rospy.set_param("controller_request", 1)
            wait_till_done("controller_done", "Done Init Controller", rate)
            rospy.set_param("controller_done", 0)

        # sleep(30)
        # gain - 0.4 - angular, same for last state angular, max - 0.4
        # start_octomap_server()

        rospy.logwarn ("Requesting Object")
        rospy.set_param("object_recognition_request", 1)
        wait_till_done("object_recognition_done", "Done Object", rate)

        if '--include_table_pose' in myargv:
            rospy.logwarn ("Requesting Table Segmentation")
            rospy.set_param("table_segmentation_request", 1)
            wait_till_done("table_segmentation_done", "Done Table Segmentation", rate)

        rospy.logwarn ("Requesting Grasp")
        rospy.set_param("grasp_request", 1)
        wait_till_done("grasp_done", "Done Grasp", rate)

        rospy.logwarn ("Requesting Planner")
        rospy.set_param("walker_planner_mode", "FULLBODY")
        rospy.set_param("walker_planner_request", 1)
        wait_till_done("walker_planner_done", "Done Planner", rate)

        rospy.logwarn ("Requesting Controller")
        rospy.set_param("controller_request", 1)
        wait_till_done("controller_done", "Done Controller", rate)

        rospy.logwarn ("Done Executing State Machine")
        setup_variables()

    except KeyboardInterrupt:
        print('interrupted!')
