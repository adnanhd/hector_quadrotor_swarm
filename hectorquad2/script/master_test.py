#!/usr/bin/env python

import rospy
import math
import tf.transformations
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
epsilon = 0.1


def PoseToStr(pose):
    return '[x:%2.1f y:%2.1f z:%2.1f][x:%2.1f y:%2.1f z:%2.1f]' % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z)


def TwistToStr(twist):
    return '[x:%2.1f y:%2.1f z:%2.1f][x:%2.1f y:%2.1f z:%2.1f]' % (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z)


def update_position(msg, pose):
    pose[0] = msg.pose.position.x
    pose[1] = msg.pose.position.y
    pose[2] = msg.pose.position.z
    pose[3] = msg.pose.orientation.z
    rospy.loginfo_throttle(
        0.1, 'roll:%+3.2f pitch:%+3.2f yaw:%+3.2f x:%+3.2f y:%3.2f z:%+3.2f w:%+3.2f' % (Quad2Euler(pose) + tuple(pose)))


def Quad2Euler(quad):  # quad is of type PoseStamped().pose
    '''
    EulerAngles ToEulerAngles(Quaternion q) {
        EulerAngles angles;

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        angles.roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles.pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        angles.yaw = std::atan2(siny_cosp, cosy_cosp);

        return angles;
    }
    '''
    x = quad[0]
    y = quad[1]
    z = quad[2]
    w = quad[3]

    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if (abs(sinp) >= 1):
        # use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


if __name__ == '__main__':
    waypoints = [(10, 10, 25), (10, -10, 25), (-10, -10, 25), (-10, 10, 25)]
    position = [0, 0, 0, 0]
    velocity = [0, 0, 0, 0]
    msg = Twist()
    i = 1

    rospy.init_node('master', anonymous=True)
    rospy.Subscriber('ground_truth_to_tf/pose',
                     PoseStamped, update_position, position)

    pub = rospy.Publisher('cmd_vel',
                          Twist, queue_size=10)

    rate = rospy.Rate(10)

    rospy.spin()
