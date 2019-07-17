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
from std_msgs.msg import String, Empty
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ImageSensor_msg
from geometry_msgs.msg import PoseStamped
from dope.msg import PoseCNNMsg
from std_msgs.msg import String
# from sensor_msgs.msg import Image

from PIL import Image
from PIL import ImageDraw

# Import DOPE code
rospack = rospkg.RosPack()
g_path2package = rospack.get_path('dope')
sys.path.append("{}/src/inference".format(g_path2package))
from cuboid import *
from detector import *

### Global Variables
g_bridge = CvBridge()
g_img = None
g_draw = None


### Basic functions
def __image_callback(msg):
    '''Image callback'''
    # rospy.logerr("test")
    global g_img
    g_img = g_bridge.imgmsg_to_cv2(msg, "rgb8")
    # cv2.imwrite('img.png', cv2.cvtColor(g_img, cv2.COLOR_BGR2RGB))  # for debugging


### Code to visualize the neural network output

def DrawLine(point1, point2, lineColor, lineWidth):
    '''Draws line on image'''
    global g_draw
    if not point1 is None and point2 is not None:
        g_draw.line([point1,point2], fill=lineColor, width=lineWidth)

def DrawDot(point, pointColor, pointRadius):
    '''Draws dot (filled circle) on image'''
    global g_draw
    if point is not None:
        xy = [
            point[0]-pointRadius,
            point[1]-pointRadius,
            point[0]+pointRadius,
            point[1]+pointRadius
        ]
        g_draw.ellipse(xy,
            fill=pointColor,
            outline=pointColor
        )

def DrawCube(points, color=(255, 0, 0)):
    '''
    Draws cube with a thick solid line across
    the front top edge and an X on the top face.
    '''

    lineWidthForDrawing = 2

    # draw front
    DrawLine(points[0], points[1], color, lineWidthForDrawing)
    DrawLine(points[1], points[2], color, lineWidthForDrawing)
    DrawLine(points[3], points[2], color, lineWidthForDrawing)
    DrawLine(points[3], points[0], color, lineWidthForDrawing)

    # draw back
    DrawLine(points[4], points[5], color, lineWidthForDrawing)
    DrawLine(points[6], points[5], color, lineWidthForDrawing)
    DrawLine(points[6], points[7], color, lineWidthForDrawing)
    DrawLine(points[4], points[7], color, lineWidthForDrawing)

    # draw sides
    DrawLine(points[0], points[4], color, lineWidthForDrawing)
    DrawLine(points[7], points[3], color, lineWidthForDrawing)
    DrawLine(points[5], points[1], color, lineWidthForDrawing)
    DrawLine(points[2], points[6], color, lineWidthForDrawing)

    # draw dots
    DrawDot(points[0], pointColor=color, pointRadius = 4)
    DrawDot(points[1], pointColor=color, pointRadius = 4)

    # draw x on the top
    DrawLine(points[0], points[5], color, lineWidthForDrawing)
    DrawLine(points[1], points[4], color, lineWidthForDrawing)


def run_dope_node(params, freq=5):
    '''Starts ROS node to listen to image topic, run DOPE, and publish DOPE results'''

    global g_img
    global g_draw

    pubs = {}
    pubs_world = {}
    models = {}
    pnp_solvers = {}
    pub_dimension = {}
    draw_colors = {}

    # Initialize parameters
    matrix_camera = np.zeros((3,3))
    matrix_camera[0,0] = params["camera_settings"]['fx']
    matrix_camera[1,1] = params["camera_settings"]['fy']
    matrix_camera[0,2] = params["camera_settings"]['cx']
    matrix_camera[1,2] = params["camera_settings"]['cy']
    matrix_camera[2,2] = 1
    dist_coeffs = np.zeros((4,1))

    if "dist_coeffs" in params["camera_settings"]:
        dist_coeffs = np.array(params["camera_settings"]['dist_coeffs'])
    config_detect = lambda: None
    config_detect.mask_edges = 1
    config_detect.mask_faces = 1
    config_detect.vertex = 1
    config_detect.threshold = 0.5
    config_detect.softmax = 1000
    config_detect.thresh_angle = params['thresh_angle']
    config_detect.thresh_map = params['thresh_map']
    config_detect.sigma = params['sigma']
    config_detect.thresh_points = params["thresh_points"]
    config_detect.device = params["device"]
    # For each object to detect, load network model, create PNP solver, and start ROS publishers
    for model in params['weights']:
        models[model] =\
            ModelData(
                name=model,
                net_path=g_path2package + "/weights/" + params['weights'][model],
                device=params['device']
            )
        models[model].load_net_model()

        draw_colors[model] = \
            tuple(params["draw_colors"][model])
        pnp_solvers[model] = \
            CuboidPNPSolver(
                model,
                matrix_camera,
                Cuboid3d(params['dimensions'][model]),
                dist_coeffs=dist_coeffs
            )
        pubs[model] = \
            rospy.Publisher(
                '{}/pose_{}'.format(params['topic_publishing'], model),
                PoseStamped,
                queue_size=10
            )
        pubs_world[model] = \
            rospy.Publisher(
                '{}/pose_world_{}'.format(params['topic_publishing'], model),
                PoseStamped,
                queue_size=10
            )
        pub_dimension[model] = \
            rospy.Publisher(
                '{}/dimension_{}'.format(params['topic_publishing'], model),
                String,
                queue_size=10
            )

        pose_world_object = \
            rospy.Publisher(
              '{}/pose_world_object'.format(params['topic_publishing']),
              PoseStamped,
              queue_size=10
            )

    # Start ROS publisher
    pub_rgb_dope_points = \
        rospy.Publisher(
            params['topic_publishing']+"/rgb_points",
            ImageSensor_msg,
            queue_size=10
        )

    # Starts ROS listener
    rospy.Subscriber(
        topic_cam,
        ImageSensor_msg,
        __image_callback
    )

    posecnn_pub = rospy.Publisher('posecnn_result', PoseCNNMsg, queue_size=1)

    # Initialize ROS node
    # rospy.init_node('dope_vis', anonymous=True)

    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(freq)
    # rospy.set_param('use_sim_time', True)
    # rospy.set_param('use_sim_time', 'true')

    rospy.logwarn ("Running DOPE...  (Listening to camera topic: '{}')".format(topic_cam))
    rospy.logwarn ("Ctrl-C to stop")
    results = []
    msg = PoseStamped()
    count = 0


    while not rospy.is_shutdown():
        # tf_broadcaster.sendTransform((0, 0, 0),
        #                  # (0.7071, 0, 0, -0.7071),
        #                  tf.transformations.quaternion_from_euler(0, math.pi, 0),
        #                  rospy.Time.now(),
        #                  "/dope",     # child
        #                  "/camera_rgb_optical_frame"      # parent
        #                  )
        if g_img is not None:
            # Copy and draw image


            img_copy = g_img.copy()
            im = Image.fromarray(img_copy)
            g_draw = ImageDraw.Draw(im)
            labels = []
            if rospy.get_param("object_recognition_request") == 1:
                results = []
                msg = PoseStamped()
                pose_in_world = None
                rospy.logwarn ("Received object recognition request")
                # rospy.set_param("object_recognition_request", 0)
                for m in models:
                    # Detect object
                    labels.append(models[m].get_name())
                    results = ObjectDetector.detect_object_in_image(
                                models[m].net,
                                pnp_solvers[m],
                                g_img,
                                config_detect
                                )

                poses = []
                print(len(results))
                for i_r, result in enumerate(results):
                    if result["location"] is None:
                        continue
                    loc = result["location"]
                    ori = result["quaternion"]
                    # msg = PoseStamped()
                    msg.header.frame_id = params["frame_id"]
                    # msg.header.frame_id = "/dope"
                    # msg.header.frame_id = "/camera_rgb_optical_frame"
                    # msg.header.stamp = rospy.Time.now()
                    CONVERT_SCALE_CM_TO_METERS = 100
                    # loc[1] = 100 - loc[1]

                    msg.pose.position.x = loc[0] / CONVERT_SCALE_CM_TO_METERS
                    msg.pose.position.y = loc[1] / CONVERT_SCALE_CM_TO_METERS
                    msg.pose.position.z = loc[2] / CONVERT_SCALE_CM_TO_METERS
                    # print("X : %f Y: %f Z: %f" % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
                    msg.pose.orientation.x = ori[0]
                    msg.pose.orientation.y = ori[1]
                    msg.pose.orientation.z = ori[2]
                    msg.pose.orientation.w = ori[3]

                    print("Found Object 1");


                    try:
                        rospy.logwarn ("Done object recognition request")
                        translation, rotation = tf_listener.lookupTransform('/map', params["frame_id"], rospy.Time(0))
                        pose_in_world = tf_listener.transformPose("/map", msg)
                        # pubs_world[m].publish(pose_in_world)
                        # print(pose_in_world).
                        rospy.logwarn(pose_in_world)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
                    # Draw the cube
                    if None not in result['projected_points']:
                        points2d = []
                        for pair in result['projected_points']:
                            points2d.append(tuple(pair))
                        DrawCube(points2d, draw_colors[m])

            # Publish the image with results overlaid


        if len(results) > 0:
            # print("Found Object 2");
            rospy.set_param("object_recognition_request", 0)
            rospy.set_param("object_recognition_done", 1)

            if None not in result['projected_points']:
                points2d = []
                for pair in result['projected_points']:
                    points2d.append(tuple(pair))
                DrawCube(points2d, draw_colors[m])

            for m in models:
                pubs[m].publish(msg)
                pub_dimension[m].publish(str(params['dimensions'][m]))

                if pose_in_world is not None:
                    # rospy.logwarn(pose_in_world)
                    pubs_world[m].publish(pose_in_world)
                    pose_world_object.publish(pose_in_world)


        rate.sleep()


if __name__ == "__main__":
    '''Main routine to run DOPE'''

    rospy.init_node('dope_vis', anonymous=True, log_level=rospy.INFO)
    rospy.set_param("object_recognition_request", 0)
    config_name = "config_pose.yaml"
    # if len(sys.argv) > 1:
    #     config_name = sys.argv[1]
    # else:
    #     config_name = "config_pose.yaml"
    rospack = rospkg.RosPack()
    params = None
    yaml_path = g_path2package + '/config/{}'.format(config_name)
    # rospy.logerr(yaml_path)
    with open(yaml_path, 'r') as stream:
        try:
            rospy.logwarn("Loading DOPE parameters from '{}'...".format(yaml_path))
            params = yaml.load(stream)
            rospy.logwarn('    Parameters loaded.')
        except yaml.YAMLError as exc:
            rospy.logerr(exc)

    topic_cam = params['topic_camera']

    try :
        run_dope_node(params)
    except rospy.ROSInterruptException:
        pass
