#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped


def update_position(msg, position):
    position[0] = msg.pose.position.x
    position[1] = msg.pose.position.y
    position[2] = msg.pose.position.z


if __name__ == '__main__':
    waypoints = [(10, 10, 25), (10, -10, 25), (-10, -10, 25), (-10, 10, 25)]
    position = [0, 0, 0]
    velocity = [0, 0, 0]
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

        if (velocity == [0, 0, 0]):
            i = (i+1) % len(waypoints)

        msg.linear.x = velocity[0]
        msg.linear.y = velocity[1]
        msg.linear.z = velocity[2]

        pub.publish(msg)

        rate.sleep()
