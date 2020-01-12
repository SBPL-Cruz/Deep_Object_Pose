#!/usr/bin/env python

# Copyright (c) 2018 NVIDIA Corporation. All rights reserved.
# This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
# https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

"""
This file starts a ROS node to run DOPE, 
listening to an image topic and publishing poses.
"""

from __future__ import print_function

import cv2
import message_filters
import numpy as np
import resource_retriever
import rospy
import tf.transformations
from PIL import Image
from PIL import ImageDraw
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, CameraInfo, Image as ImageSensor_msg
from std_msgs.msg import String

def image_info_cloud_callback(image_msg, camera_info, cloud_in):
    print("Received image_info_cloud_callback in sync message")

def image_cloud_callback(image_msg, cloud_in):
    print("Received image_cloud_callback in sync message")


def main():

    # Initialize ROS node
    rospy.init_node('dope_sync_test')
    print("Init node")
    image_sub = message_filters.Subscriber(
            "/camera/rgb/image_color",
            ImageSensor_msg
        )
    info_sub = message_filters.Subscriber(
        "/camera/rgb/camera_info",
        CameraInfo
    )
    cloud_sub = message_filters.Subscriber(
        "/camera/depth/points",
        PointCloud2
    )
    ts = message_filters.TimeSynchronizer([image_sub, info_sub, cloud_sub], 1000)
    # ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub, cloud_sub], 1, 10, 0.1)

    ts.registerCallback(image_info_cloud_callback)

    ts = message_filters.TimeSynchronizer([image_sub, cloud_sub], 1000)
    # ts = message_filters.ApproximateTimeSynchronizer([image_sub, cloud_sub], 1, 10, 0.1)

    ts.registerCallback(image_cloud_callback)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
