#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped


def update_position(msg, position):
    position[0] = msg.pose.position.x
    position[1] = msg.pose.position.y
    position[2] = msg.pose.position.z
    position[3] = math.atan2(2.0 * (msg.pose.orientation.w * msg.pose.orientation.z + msg.pose.orientation.x * msg.pose.orientation.y),
                             1.0 - 2.0 * (msg.pose.orientation.y * msg.pose.orientation.y + msg.pose.orientation.z * msg.pose.orientation.z))


def coeff(a, b):
    if abs(a-b) < 1:
        return 0
    elif(a < b):
        return 1
    else:
        return -1


def PoseToStr(pose):
    return '[x:%2.1f y:%2.1f z:%2.1f][x:%2.1f y:%2.1f z:%2.1f]' % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z)


if __name__ == '__main__':
    waypoints = [(10, 10, 25), (10, -10, 25), (-10, -10, 25), (-10, 10, 25)]
    position = [0, 0, 0, 0]
    velocity = [0, 0, 0, 0]
    msg = Twist()
    i = 1

    rospy.init_node('master', anonymous=True)
    rospy.Subscriber('ground_truth_to_tf/pose',
                     PoseStamped, update_position, position)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        for dim in [0, 1, 2]:
            if abs(waypoints[i][dim] - position[dim]) > 0.25:
                velocity[dim] = (waypoints[i][dim] - position[dim]) / 2.0
            else:
                velocity[dim] = 0

        if (velocity[0:3] == [0, 0, 0]):
            i = (i+1) % len(waypoints)

        msg.linear.x = velocity[0]
        msg.linear.y = velocity[1]
        msg.linear.z = velocity[2]
        msg.angular.z = velocity[3]

        pub.publish(msg)

        rate.sleep()
