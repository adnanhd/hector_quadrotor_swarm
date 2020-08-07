#!/usr/bin/env python

import rospy
import math
import copy
from geometry_msgs.msg import Twist, Quaternion, PoseStamped

msg_twist = [0, 0, 0, 1.0]
msg_pose = PoseStamped()


def get_position(msg):
    global msg_pose
    msg_pose = copy.deepcopy(msg)


def set_position(msg):
    global msg_twist
    global msg_pose
    repulsive_constant = 5.0
    attractive_constant = 10.0
    x = msg_pose.pose.position.x - msg.pose.position.x
    y = msg_pose.pose.position.y - msg.pose.position.y
    z = msg_pose.pose.position.z - msg.pose.position.z

    rospy.logerr_throttle(5.0, '%s <--> %s %3.2f x=%2.1f y=%2.1f z=%2.1f' % (
        msg_pose.header.frame_id, msg.header.frame_id, math.sqrt(x*x + y*y + z*z), x, y, z))
    if (repulsive_constant > math.sqrt(x*x + y*y + z*z)):
        msg_twist[0] += x
        msg_twist[1] += y
        msg_twist[2] += z
        msg_twist[3] += 1
    elif (attractive_constant < math.sqrt(x*x + y*y + z*z)):
        msg_twist[0] -= x
        msg_twist[1] -= y
        msg_twist[2] -= z
        msg_twist[3] += 1


if __name__ == '__main__':
    num_of_drones = len(rospy.myargv()) - 1
    node_name = '' if num_of_drones == 0 else '/' + rospy.myargv()[1]

    rospy.init_node('driver', anonymous=True)

    rospy.Subscriber(node_name + '/ground_truth_to_tf/pose',
                     PoseStamped, get_position)

    for arg in rospy.myargv()[2:]:
        rospy.Subscriber('/' + arg + '/ground_truth_to_tf/pose',
                         PoseStamped, set_position)

    pub = rospy.Publisher(node_name + '/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)
    msg = Twist()

    while not rospy.is_shutdown():
        msg.linear.x = msg_twist[0] / msg_twist[3]
        msg.linear.y = msg_twist[1] / msg_twist[3]
        msg.linear.z = msg_twist[2] / msg_twist[3]

        pub.publish(msg)
        rospy.loginfo_throttle(5.0, '%s: %2.1f %2.1f %2.1f' % (
            rospy.get_namespace(), msg.linear.x, msg.linear.y, msg.linear.z))
        msg_twist = [0, 0, 0, 1.0]
        rate.sleep()
