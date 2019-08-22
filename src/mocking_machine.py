#!/usr/bin/env python
import rospy
import rosparam
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, Pose


def create_pose_from_vectors(position, orientation):
    pose = Pose()
    (
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ) = position
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = orientation
    return pose


def publish_grasp_pose_tf(pose, frame_id='base_footprint', child_frame='grasp'):
    broadcaster = tf.TransformBroadcaster()
    broadcaster.sendTransform(
        pose.position,
        pose.orientation,
        rospy.Time.now(),
        child_frame,
        frame_id
    )


def wait_till_done_and_publish(param, done_msg, pub, msg, rate=10):
    rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        try:
            pub.publish(msg)
            if (rospy.get_param(param) == 1):
                rospy.logwarn(done_msg)
                break
            rate.sleep()
        except KeyboardInterrupt:
            print('interrupted!')


def publishing_till_received(param, done_msg, pub, msg, rate=10):
    rate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        try:
            if (rospy.get_param(param) == 0):
                rospy.logwarn(done_msg)
                break
            pub.publish(msg)
            rate.sleep()
        except KeyboardInterrupt:
            print('interrupted!')


def plan_to_custom_goal(pose, frame_id="base_footprint"):
    rospy.logwarn("Requesting Multi-Step Planner")
    rospy.set_param("walker_planner_request", 1)
    grasp_publisher = rospy.Publisher('/Grasp', PoseStamped, queue_size=10)
    grasp = PoseStamped()
    grasp.pose = pose
    grasp.header.frame_id = frame_id
    # publishing_till_received(param="walker_planner_request", 
    #                            done_msg="Goal received!",
    #                            pub=grasp_publisher,
    #                            msg=grasp
    #                            )
    wait_till_done_and_publish(param="walker_planner_done", 
                               done_msg="Planner Done!",
                               pub=grasp_publisher,
                               msg=grasp
                               )
    rospy.set_param("walker_planner_done", 0)


def setup_variables():
    rospy.set_param("walker_planner_request", 0)
    rospy.set_param("walker_planner_done", 0)
    rospy.set_param("from_current_state", 1)
    rospy.set_param("controller_request", 0)
    rospy.set_param("controller_done", 0)


def main():
    rospy.init_node('mocking_machine')
    setup_variables()

    # go to dummy goal from current robot state
    position = np.array([0.25, -0.40, 0.80])
    orientation = np.array([0.0, 0.0, 0.7071068, 0.7071068])
    testing_goal_from_current = create_pose_from_vectors(position, orientation)
    plan_to_custom_goal(testing_goal_from_current)
    # go to dummy goal from the last robot state
    rospy.set_param("from_current_state", 0)
    position = np.array([0.25, -0.30, 0.80])
    testing_goal_from_last = create_pose_from_vectors(position, orientation)
    plan_to_custom_goal(testing_goal_from_last)

if __name__ == '__main__':
    main()