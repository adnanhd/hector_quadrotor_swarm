#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped


def Quad2Euler(q):
    """
    This function transforms from quernion to euler

    TODO: Extended description of function.

    Parameters
    ----------
    q : PoseStamped().pose.orientation
        This is an orientation vector in queternion form

    Returns
        The euler form of the parameter q
    -------
    int
        Description of return value
    """
    # @see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if (abs(sinp) >= 1):
        # use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Transform the q of the form PoseStamped().pose.orientation to angle
    # by applying atan2(y,x) => float angle from math library.
    return (roll, pitch, yaw)


def update_position(msg, position):
    position[0] = msg.pose.position.x
    position[1] = msg.pose.position.y
    position[2] = msg.pose.position.z
    position[3] = Quad2Euler(msg.pose.orientation)[2]
    rospy.loginfo('yaw: %3.2f' % position[3])


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

    rospy.spin()

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
