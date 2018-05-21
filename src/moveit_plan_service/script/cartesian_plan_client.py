#!/usr/bin/env python

import sys
import rospy
from moveit_plan_service.srv import *
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def arm_cartesian_plan(pose):
    rospy.wait_for_service('cartesian_arm_plan')
    try:
        cart_arm_plan = rospy.ServiceProxy('cartesian_arm_plan', CartesianArmPlan)
        cart_arm_plan(pose)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s use default value: [x y z roll pitch yaw]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 7:
        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.pose.position.x = float(sys.argv[1])
        pose.pose.position.y = float(sys.argv[2])
        pose.pose.position.z = float(sys.argv[3])
        quat = quaternion_from_euler(float(sys.argv[4]),
        float(sys.argv[5]), float(sys.argv[6]))
        pose.pose.orientation.w = quat[0]
        pose.pose.orientation.x = quat[1]
        pose.pose.orientation.y = quat[2]
        pose.pose.orientation.z = quat[3]

    else:
        print usage()
        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.pose.position.x = 0.45
        pose.pose.position.y = -0.3
        pose.pose.position.z = 0.46
        # quat = quaternion_from_euler(-0.011, 1.57, 0.037)
        # pose.pose.orientation.w = float(quat[0])
        # pose.pose.orientation.x = float(quat[1])
        # pose.pose.orientation.y = float(quat[2])
        # pose.pose.orientation.z = float(quat[3])
        pose.pose.orientation.w = 1
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        # sys.exit(1)
    arm_cartesian_plan(pose)