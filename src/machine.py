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
from geometry_msgs.msg import PoseStamped, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers 
from time import sleep
from sensor_msgs.msg import JointState

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

def release_object():
    # go to end position and release
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (0.282, -0.30, 1.0)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w \
        = (0.0, 0.0, 0.865, 0.500)
    go_to_custom_goal(pose, rate, "FULLBODY", controller=True)
    rospy.set_param("hand_open_request", 1)

def control_execution(skipNum, rate):
    rospy.logwarn ("Requesting Controller")
    rospy.set_param("skip_states", skipNum)
    rospy.set_param("controller_request", 1)
    wait_till_done("controller_done", "Done Init Controller", rate)
    rospy.set_param("controller_done", 0)


def setup_variables():
    rospy.set_param("object_recognition_request", 0)
    rospy.set_param("object_recognition_done", 0)
    rospy.set_param("conveyor_segmentation_request", 0)
    rospy.set_param("conveyor_segmentation_done", 0)
    rospy.set_param("grasp_request", 0)
    rospy.set_param("grasp_done", 0)
    rospy.set_param("walker_planner_request", 0)
    rospy.set_param("walker_planner_done", 0)
    rospy.set_param("controller_request", 0)
    rospy.set_param("controller_done", 0)
    rospy.set_param("trac_ik_request", 0)
    rospy.set_param("trac_ik_done", 0)
    rospy.set_param("lift_request", 0)
    rospy.set_param("skip_states", 1)
    rospy.set_param("hand_open_request", 1)

def load_yaml(yaml_path, ros_param_name):

    # with open(yaml_path, 'r') as stream:
    #     try:
    #         rospy.logwarn("Loading Planner Home state parameters from '{}'...".format(yaml_path))
    #         params = yaml.load(stream)
    #         rospy.set_param(ros_param_name, params['initial_configuration']['joint_state'])
    #     except yaml.YAMLError as exc:
    #         rospy.logerr(exc)
    paramlist=rosparam.load_file(yaml_path)
    for params, ns in paramlist:
        rosparam.upload_params(ns,params)


def get_transform_pose(source, target, pose):
    msg = PoseStamped()
    msg.header.frame_id = source
    msg.header.stamp = rospy.Time(0)
    msg.pose = pose
    tf_listener = tf.TransformListener()
    now = rospy.Time.now()
    rospy.logwarn("Waiting for transform")
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            translation, rotation = tf_listener.lookupTransform(target, source, rospy.Time(0))
            out_msg = tf_listener.transformPose(target, msg)
            rospy.logwarn("Transformed pose found")
            return out_msg.pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()



def go_to_custom_goal(pose, rate, mode, controller=True):
    grasp_publisher = rospy.Publisher(
        '/pose_filtered',
        PoseStamped,
        queue_size=10
    )
    msg = PoseStamped()
    msg.header.frame_id = "base_footprint"
    msg.header.stamp = rospy.Time.now()
    msg.pose = pose
    rospy.logwarn ("Requesting Planner")
    # rospy.set_param("walker_planner_mode", mode)
    rospy.set_param("walker_planner_request", 1)
    wait_till_done_and_publish("walker_planner_done", "Done Init Planner", rate, grasp_publisher, msg)
    rospy.set_param("walker_planner_done", 0)

    if controller:
        control_execution(0,rate)

def joint_state_callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
    joint_position_list = [{'name': 'x', 'position': 0.0}, {'name': 'y', 'position': 0.0}, {'name': 'theta', 'position': 0.0}]
    for name, position in zip(data.name, data.position):
        joint_position_list.append({
            'name' : name,
            'position' : position
        })
    # print(joint_position_list)
    rospy.set_param('initial_configuration/joint_state', joint_position_list)

#TODO: hardcode the threshold 1.25m and 0.75m
def object_pose_callback(pose):
    #target_publiser = rospy.Publisher('/Grasp', PoseStamped, queue_size=2)
    #TODO: assuming x is the axis that's changing
    #if marker.markers: 

    #pst = marker.markers[0].pose.pose.position.y;
    pst = pose.pose.position.y;
    if pst < 0.60 and rospy.get_param("object_recognition_done") == 0:
    	rospy.set_param("object_recognition_done",1);
        #target_publiser.publish(pose);
    #elif pst < 0.75 and rospy.get_param("object_recognition_done") == 1:
       	#rospy.set_param("object_recognition_done",2);
        #target_publiser.publish(pose);
	#print("Pose published")

def object_recognition_buffer(current_state, done_msg, rate):
    while not rospy.is_shutdown():
        if (rospy.get_param("object_recognition_done") != current_state ):
            rospy.logwarn (done_msg)
            break
        else:
            #rospy.Subscriber("/ar_pose_marker", AlvarMarkers ,object_pose_callback)
            rospy.Subscriber("/pose_filtered",  PoseStamped, object_pose_callback)
            #rate.sleep()

if __name__ == "__main__":

    try:
        rospy.init_node('cruiser_state_machine', anonymous=True, log_level=rospy.INFO)
        rate = rospy.Rate(50)
        setup_variables()

        myargv = rospy.myargv(argv=sys.argv)
        print(myargv)

        '''
        if '--include_conveyor_pose' in myargv:
            rospy.logwarn ("Requesting Converyor Segmentation")
            rospy.set_param("Conveyor_segmentation_request", 1)
            wait_till_done("Conveyor_segmentation_done", "Done Conveyor Segmentation", rate)
        '''

        #TODO: Don't know if the planner need this
        if '--custom_home_state' in myargv:
            # Use the planner to go to a custom home location
            rospy.logwarn ("Requesting Hand to Custom Home State")

            # First read the goal state which is at hand by the side
            config_name = "walker_goal.yaml"
            yaml_path = '{}/experiments/{}'.format(planner_path, config_name)

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (0.304, -0.507, 0.795)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w \
                 = (0.282, 0.155, 0.702, 0.635)

            #TODO: map is not used in the world, need another transformation
            pose_in_map = get_transform_pose('base_footprint', 'base_footprint', pose)
            load_yaml(yaml_path, "initial_configuration/joint_state")
            go_to_custom_goal(pose_in_map, rate, "FULLBODY", controller=False)

        while not rospy.is_shutdown():

            rospy.logwarn ("Requesting Object")
            rospy.set_param("object_recognition_request", 0)
            object_recognition_buffer(rospy.get_param("object_recognition_done"),"Done Object", rate)
            rospy.set_param("walker_planner_request", 1)
            
            # rospy.logwarn ("Requesting Grasp")
            # rospy.set_param("grasp_request", 1)
            # wait_till_done("grasp_done", "Done Grasp", rate)

            rospy.logwarn ("Requesting Planner")
            wait_till_done("walker_planner_done", "Done Planner", rate)
            rospy.set_param("walker_planner_done", 0)

            control_execution(1, rate)

            # Delay as we do not know when obj arrives
            rospy.sleep(5.0)
    	    rospy.set_param("hand_close_request", 1)

            release_object();   

        rospy.logwarn ("Done Executing State Machine")
        setup_variables()

    except KeyboardInterrupt:
        print('interrupted!')
