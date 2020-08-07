#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped

x = 0
y = 0
z = 0
w = 0


def callback(data):
    global x
    global y
    global z
    global w
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    w = math.atan2(2.0 * (data.pose.orientation.w * data.pose.orientation.z + data.pose.orientation.x * data.pose.orientation.y),
                   1.0 - 2.0 * (data.pose.orientation.y * data.pose.orientation.y + data.pose.orientation.z * data.pose.orientation.z))


def coeff(a, b):
    if abs(a-b) < 1:
        return 0
    elif(a < b):
        return 1
    else:
        return -1


if __name__ == '__main__':
    i = 0
    waypoints = [(5, 5, 5), (-1, -1, 2), (10, 10, 10)]

    rospy.init_node('node', anonymous=True)
    for drone_num in range(1, 5):
        pub = rospy.Publisher('/drone'+str(drone_num) +
                              '/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = Twist()
        if(i == len(waypoints)):
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.z = 0
        else:
            distancex = abs(x - waypoints[i][0])
            distancey = abs(y - waypoints[i][1])
            distancez = abs(z - waypoints[i][2])

            msg.linear.x = 0.50 * distancex * coeff(x, waypoints[i][0])
            msg.linear.y = 0.50 * distancey * coeff(y, waypoints[i][1])
            msg.linear.z = 0.50 * distancez * coeff(z, waypoints[i][2])

            if(coeff(x, waypoints[i][0]) == 0 and coeff(y, waypoints[i][1]) == 0 and coeff(z, waypoints[i][2]) == 0):
                i = i + 1

            msg.angular.z = - w / 8

        rospy.loginfo(str(distancex) + " " +
                      str(distancey) + " " + str(distancez))
        pub.publish(msg)
        rate.sleep()
