#!/usr/bin/env python

import rospy
import math
import copy
from geometry_msgs.msg import Twist, Quaternion, PoseStamped

msg_twist = Twist()
msg_pose = PoseStamped()


def get_position(msg):
    global msg_pose
    msg_pose = copy.deepcopy(msg)


def set_position(msg):
    global msg_twist
    global msg_pose
    repulsive_constant = 10.0
    attractive_constant = 20.0
    x = msg_pose.pose.orientation.x - msg.pose.orientation.x
    y = msg_pose.pose.orientation.y - msg.pose.orientation.y
    z = msg_pose.pose.orientation.z - msg.pose.orientation.z
    if (not (repulsive_constant < math.sqrt(x*x + y*y + z*z) < attractive_constant)):
        msg_twist.linear.x += 1.0 - x/10 
        msg_twist.linear.y += 1.0 - y/10
        msg_twist.linear.z += 1.0 - z/10

if __name__ == '__main__':
    arglen = len(rospy.myargv())
    node_name = 'drone' if arglen < 2 else rospy.myargv()[1]

    rospy.init_node(node_name, anonymous=True)

    rospy.Subscriber(node_name + '/ground_truth_to_tf/pose',
                     PoseStamped, get_position)

    for arg in rospy.myargv()[2:]:
        rospy.Subscriber(arg + '/ground_truth_to_tf/pose',
                         PoseStamped, set_position)

    pub = rospy.Publisher(node_name + '/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = Twist()
        rate.sleep()
