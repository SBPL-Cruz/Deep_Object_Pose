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
planner_path = rospack.get_path('cruzr_planner')


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
        pub.publish(msg)
        if (rospy.get_param(param) == 1):
            rospy.logwarn (done_msg)
            break
        rate.sleep()

def wait_till_done(param, done_msg, rate):
    while not rospy.is_shutdown():
        if (rospy.get_param(param) == 1):
            rospy.logwarn (done_msg)
            break
        rate.sleep()

if __name__ == "__main__":
    '''Main routine to run DOPE'''



    rospy.init_node('cruiser_state_machine', anonymous=True, log_level=rospy.INFO)
    rate = rospy.Rate(5)
    rospy.set_param("object_recognition_request", 0)
    rospy.set_param("object_recognition_done", 0)
    rospy.set_param("table_segmentation_request", 0)
    rospy.set_param("table_segmentation_done", 0)
    rospy.set_param("grasp_request", 0)
    rospy.set_param("grasp_done", 0)
    rospy.set_param("cruzr_planner_request", 0)
    rospy.set_param("cruzr_planner_done", 0)
    rospy.set_param("controller_request", 0)
    rospy.set_param("controller_done", 0)

    myargv = rospy.myargv(argv=sys.argv)
    print(myargv)

    grasp_publisher = rospy.Publisher(
        '/Grasps',
        PoseStamped,
        queue_size=10
    )
    msg = PoseStamped()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()

    if '--target' in myargv:
        # Table location
        print("Moving to table location")
        msg.pose.position.x = 1.259194
        msg.pose.position.y = -0.307047
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = -0.635
        msg.pose.orientation.w = 0.773
    elif '--initial' in myargv:
        # Initial location
        print("Moving to initial location")
        msg.pose.position.x = 0.969
        msg.pose.position.y = 1.767
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = -0.574
        msg.pose.orientation.w = 0.819
        # 1.259194, -0.417047, -1.664656
    else:
        # Table location
        print("Moving to table location")
        msg.pose.position.x = 1.259194
        msg.pose.position.y = -0.307047
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = -0.574
        msg.pose.orientation.w = 0.819

    rospy.logwarn ("Requesting Initial Planner")
    rospy.set_param("cruzr_planner_mode", "BASE")
    rospy.set_param("cruzr_planner_request", 1)


    wait_till_done_and_publish("cruzr_planner_done", "Done Init Planner", rate, grasp_publisher, msg)
    rospy.set_param("cruzr_planner_done", 0)

    rospy.logwarn ("Requesting Init Controller")
    rospy.set_param("controller_request", 1)
    wait_till_done("controller_done", "Done Init Controller", rate)
    rospy.set_param("controller_done", 0)

    sleep(30)

    start_octomap_server()

    rospy.logwarn ("Requesting Object")
    rospy.set_param("object_recognition_request", 1)
    wait_till_done("object_recognition_done", "Done Object", rate)

    rospy.logwarn ("Requesting Table Segmentation")
    rospy.set_param("table_segmentation_request", 1)
    wait_till_done("table_segmentation_done", "Done Table Segmentation", rate)

    rospy.logwarn ("Requesting Grasp")
    rospy.set_param("grasp_request", 1)
    wait_till_done("grasp_done", "Done Grasp", rate)

    rospy.logwarn ("Requesting Planner")
    rospy.set_param("cruzr_planner_mode", "FULLBODY")
    rospy.set_param("cruzr_planner_request", 1)
    wait_till_done("cruzr_planner_done", "Done Planner", rate)

    rospy.logwarn ("Requesting Controller")
    rospy.set_param("controller_request", 1)
    wait_till_done("controller_done", "Done Controller", rate)

    rospy.logwarn ("Done Executing State Machine")