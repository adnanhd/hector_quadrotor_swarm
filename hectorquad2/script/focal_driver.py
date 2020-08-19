#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
epsilon = 0.001


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


def level_of_distance(distance):
    if distance < 3:
        return 7
    elif distance < 25:
        return int(8 / math.log(distance))
    else:
        return 0


if __name__ == '__main__':
    wps = [(15, 15, 10), (15, -15, 10), (-15, -15, 10), (-15, 15, 10)]
    position = [0, 0, 0, 0]
    msg = Twist()
    K_p = 2.0
    i = 0

    rospy.init_node('master', anonymous=True)
    rospy.Subscriber('ground_truth_to_tf/pose',
                     PoseStamped, update_position, position)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        delta_x = wps[i][0] - position[0]
        delta_y = wps[i][1] - position[1]
        delta_z = wps[i][2] - position[2]

        ################################
        ## HEADING ALIGNMENT BEHAVIOR ##
        ################################

        heading_vector = Twist()

        # desired z-axis rotation (yaw)
        yaw = math.atan2(delta_y, delta_x)
        # projection of desired z-axis rotation (yaw) onto x-axix
        cos_yaw = math.cos(yaw)
        # projection of desired z-axis rotation (yaw) onto y-axix
        sin_yaw = math.sin(yaw)

        heading_vector.linear.x = cos_yaw
        heading_vector.linear.y = sin_yaw
        heading_vector.angular.z = yaw

        ################################
        ## PROXIMAL CONTROL BEHAVIOR  ##
        ################################

        proximal_vector = Twist()

        o_k = level_of_distance(math.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z))
        o_des = 7  # Constant to reach waypoints
        C = 5.0  # Constant in the formula

        if (o_k >= o_des):
            f_k = - (o_k - o_des) ** 2 / C
        else:
            f_k = (o_k - o_des) ** 2 / C

        proximal_vector.linear.x = f_k * cos_yaw 
        proximal_vector.linear.y = f_k * sin_yaw 

        ################################
        ## DESIRED  HEADING  VECTOR   ##
        ################################

        alpha = Twist()
        beta = 2.0
        
        cos_alpha = heading_vector.linear.x + beta * proximal_vector.linear.x
        sin_alpha = heading_vector.linear.y + beta * proximal_vector.linear.y
        norm_alpha = math.sqrt(sin_alpha * sin_alpha + cos_alpha * cos_alpha)

        alpha.linear.x = cos_alpha / norm_alpha
        alpha.linear.y = sin_alpha / norm_alpha

        ################################
        ## MOTION CONTROL BEHAVIOR    ##
        ################################
        
        u_max = 5.0

        dot_product = alpha.linear.x * math.cos(position[3]) + alpha.linear.y + math.sin(position[3])

        msg.linear.x = dot_product * u_max if (dot_product > 0) else 0
        msg.linear.z = delta_z
        msg.angular.z = (yaw - position[3]) * K_p

        if (o_k == o_des):
            i = (i + 1) % len(wps)
        
        if (o_k == 0): 
            rospy.loginfo_throttle(1,'distance is unknown %3.2f' % math.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z))
            rospy.loginfo_throttle(1,'x:%3.2f y:%3.2f z:%3.2f w:%3.2f' % tuple(position))
            rospy.loginfo_throttle(1, 'x:%3.2f y:%3.2f z:%3.2f ------' % (delta_x, delta_y, delta_z))

        pub.publish(msg)

        rate.sleep()
