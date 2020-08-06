#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped 

currx = 0
curry = 0
currz = 0
yaw = 0

waypoints = [(5, 5, 5), (-1, -1, 2), (10, 10, 10)]
i = 0

rospy.init_node('node', anonymous=True) 
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def callback(data):
    global currx
    global curry
    global currz
    global yaw
    currx = data.pose.position.x
    curry = data.pose.position.y
    currz = data.pose.position.z
    yaw = math.atan2(2.0 * (data.pose.orientation.w * data.pose.orientation.z + data.pose.orientation.x * data.pose.orientation.y), 1.0 - 2.0 * (data.pose.orientation.y * data.pose.orientation.y + data.pose.orientation.z * data.pose.orientation.z))
    #rospy.loginfo(str(currx) + " " + str(curry) + " " + str(currz) + " " + str(yaw) + " ")


def coeff(a, b):
    if abs(a-b) < 1:
        return 0
    elif(a < b):
        return 1
    else:
        return -1


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
        distancex = abs(currx - waypoints[i][0])
        distancey = abs(curry - waypoints[i][1])
        distancez = abs(currz - waypoints[i][2])

        msg.linear.x = 0.50 * distancex * coeff(currx, waypoints[i][0])
        msg.linear.y = 0.50 * distancey * coeff(curry, waypoints[i][1])
        msg.linear.z = 0.50 * distancez * coeff(currz, waypoints[i][2])

        if(coeff(currx, waypoints[i][0]) == 0 and coeff(curry, waypoints[i][1]) == 0 and coeff(currz, waypoints[i][2]) == 0):
            i = i + 1

        msg.angular.z = - yaw / 8

    rospy.loginfo(str(distancex) + " " + str(distancey) + " " + str(distancez))
    pub.publish(msg)
    rate.sleep()
    