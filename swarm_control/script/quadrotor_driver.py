#!/usr/bin/env python

import rospy
import math
import copy
from geometry_msgs.msg import Twist, Quaternion, PoseStamped

msg_twist = [0, 0, 0, 1.0]
msg_pose = PoseStamped()


def focal_control(msg):
    global msg_pose
    msg_pose = msg


def proximal_control(msg):
    global msg_twist
    global msg_pose

    o_des = 10.0
    C = 100.0

    x = msg_pose.pose.position.x - msg.pose.position.x
    y = msg_pose.pose.position.y - msg.pose.position.y
    z = msg_pose.pose.position.z - msg.pose.position.z
    
    r = math.sqrt(x * x + y * y + z * z)

    if (15 < r or r < 5):
        msg_twist[3] += 1
        if x > o_des:
            msg_twist[0] -= (x-o_des)**2/C
        else:
            msg_twist[0] += (x-o_des)**2/C
        if y > o_des:
            msg_twist[1] -= (y-o_des)**2/C
        else:
            msg_twist[1] += (y-o_des)**2/C
    
    if z > 0:
        msg_twist[2] -= z**2/C
    else:
        msg_twist[2] += z**2/C


if __name__ == '__main__':
    num_of_drones = len(rospy.myargv())
    node_name = '' if num_of_drones == 0 else '/' + rospy.myargv()[1]

    rospy.init_node(rospy.myargv()[1], anonymous=True)

    rospy.Subscriber(node_name + '/ground_truth_to_tf/pose',
                     PoseStamped, focal_control)

    for arg in rospy.myargv()[2:]:
        rospy.Subscriber('/' + arg + '/ground_truth_to_tf/pose',
                         PoseStamped, proximal_control)

    pub = rospy.Publisher(node_name + '/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(5)
    msg = Twist()

    while not rospy.is_shutdown():
        msg.linear.x = msg_twist[0] / msg_twist[3]
        msg.linear.y = msg_twist[1] / msg_twist[3]
        msg.linear.z = msg_twist[2] / msg_twist[3]

        pub.publish(msg)

        msg_twist[0] = 0
        msg_twist[1] = 0
        msg_twist[2] = 0
        msg_twist[3] = 1.0
        rate.sleep()
