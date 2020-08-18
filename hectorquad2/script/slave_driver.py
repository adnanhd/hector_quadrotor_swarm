#!/usr/bin/env python

import sys
import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
epsion = 0.001


def slave_callback(msg, pose):
    """
    This is a callback function of type PoseStamped in geometry_msgs

    Updates slaves own position, stored in pose of type PoseStamped(), 
    with msg, of type PoseStamped().

    Parameters
    ----------
    msg : PoseStamped()
            This is a message of topic ground_truth_to_tf/pose containing
            location and orientation information of the agent
    pose: PoseStamped()
            This is a variable to keep up-to-date the location and orien-
            tation information of the agent

    Returns
        nothing
    -------
    """
    global slave_pose
    # Update the agent's position as soon as the topic
    # ground_truth_to_tf/pose published
    slave_pose = msg


def agent_callback(msg, args):
    """
    This is a callback function of type PoseStamped in geometry_msgs

    This function updates args[1]th element of the list args[0] of type 
    list of PoseStamped() with msg of type PoseStamped()

    Parameters
    ----------
    msg : PoseStamped()
            This is a message of topic ground_truth_to_tf/pose containing
            location and orientation information of the agent to be call-
            backed
    args: PoseStamped()
            This is a pair (2-tuple) of the list of agents' positions in 
            the swarm and the index of the agent whose pose is published 
            to be updated in the list

    Returns
        nothing
    -------
    """
    # initializing agents index and updating its position in the list
    swarm_pose = args[0]
    agent_index = args[1]
    swarm_pose[agent_index] = msg


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


def TwistToStr(twist):
    return '[x:%2.1f y:%2.1f z:%2.1f][x:%2.1f y:%2.1f z:%2.1f]' % (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z)


def PoseToStr(pose):
    return '[x:%2.1f y:%2.1f z:%2.1f][x:%2.1f y:%2.1f z:%2.1f]' % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z)


if __name__ == '__main__':
    # initialize swarm with all arguments defining swarm but excluding slave itself
    swarm = rospy.myargv()[1:]
    # initialize swarm_pose with default PoseStamped() objects
    # in order to keep position information of the agents in the swarm
    swarm_pose = [PoseStamped()] * len(swarm)
    # initialize slave_pose with a PoseStamped() object to keep postion information
    # of the slave in the swarm
    slave_pose = PoseStamped()
    # initialize slave_vel with a Twist() object to update velocity information
    # of the slave
    slave_vel = Twist()
    z_yaw = 0  # not used

    # Initialize node to controll the agent
    rospy.init_node('slave', anonymous=True)

    # Subscribe the position of the agent and the rest of agents in the swarm
    rospy.Subscriber('ground_truth_to_tf/pose',
                     PoseStamped, slave_callback, slave_pose)

    # Subscribe pose topic of the agents in the swarm to keep swarm_pose up-to-date
    # i.e., keep all agents' position in the swarm
    for agent_index in range(len(swarm)):
        rospy.Subscriber('/' + swarm[agent_index] + '/ground_truth_to_tf/pose',
                         PoseStamped, agent_callback, (swarm_pose, agent_index))

    # Publish the velocity of the agent
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # Heading vector is one pillor of the swarm and here is initialized with a Twist() object
        heading_vector = Twist()

        #################################
        ## heading alignment behaviour ##
        #################################

        cos_yaw = 0  # projection of z-axis rotation (yaw) onto x-axix
        sin_yaw = 0  # projection of z-axis rotation (yaw) onto y-axix

        for agent in swarm_pose:
            # For each position vector `agent`, of type PoseStamped() containing position information 
            # of other agents in the swarm, corresponding direction angle in xy-plane is found and ac-
            # cumulate to pitch and roll values
            z_yaw = Quad2Euler(agent.pose.orientation)[2]
            cos_yaw += math.cos(z_yaw)
            sin_yaw += math.sin(z_yaw)

        # The norm is the hypothenuse of the right angle of roll, pitch and angle: update expalanation
        norm = math.sqrt((cos_yaw * cos_yaw) + (sin_yaw * sin_yaw))

        # initialize a heading_vector
        if abs(norm) < epsion:  # epsilon is very close to zero
            heading_vector.linear.x = 0
            heading_vector.linear.y = 0
        else:
            heading_vector.linear.x = cos_yaw / norm
            heading_vector.linear.y = sin_yaw / norm

        ################################
        ## proximal control behaviour ##
        ################################

        position_vector = Twist()
        o_des = 1.75  # Constant in the formula
        force = 0
        C = 5  # Constant in the formula

        for agent_pose in swarm_pose:
            # for the current and each agent in the swarm, find the difference in between and in all three axes
            d_x = agent_pose.pose.position.x - slave_pose.pose.position.x
            d_y = agent_pose.pose.position.y - slave_pose.pose.position.y
            d_z = agent_pose.pose.position.z - slave_pose.pose.position.z

            # Find distance from the difference in three axes
            distance = math.sqrt(d_x * d_x + d_y * d_y + d_z * d_z)

            # If the distance, i.e. the agent, is far from how much it is supposed to be,
            if distance > o_des:
                # increment force f_k by ((o_des - o_k))^2 / C
                force += ((o_des - distance) ** 2) / C
            else:
                # otherwise, decrement by ((o_des - o_k))^2 / C.
                force -= ((o_des - distance) ** 2) / C

        # update position vector with corresponding position vector
        position_vector.linear.x = slave_pose.pose.position.x * force
        position_vector.linear.y = slave_pose.pose.position.y * force
        position_vector.linear.z = slave_pose.pose.position.z * force

        ############################
        ## desired heading vector ##
        ############################

        # Desired Heading Vector 'a' is initialized with a Twist() object
        a = Twist()

        # initializing the numerators in 3 axes of Desired Heading Vector and the denominator
        numerator_x = 0
        numerator_y = 0
        numerator_z = 0
        denominator = 0
        Beta = 12  # Constant in the formula

        # updating numerators with heading vector and proximal control vector in corresponding coodinate axis
        numerator_x = heading_vector.linear.x + \
            (Beta * position_vector.linear.x)
        numerator_y = heading_vector.linear.y + \
            (Beta * position_vector.linear.y)
        numerator_z = heading_vector.linear.z + \
            (Beta * position_vector.linear.z)

        # updating denominator with the norm of numerators
        denominator = math.sqrt((numerator_x * numerator_x) +
                                (numerator_y * numerator_y) + (numerator_z * numerator_z))

        if abs(denominator) < epsion:  # epsilon is very close to zero
            # if denominator is zero, then the numerators in all axes are all zero
            a.linear.x = 0
            a.linear.y = 0
            a.linear.z = 0
        else:
            a.linear.x = numerator_x / denominator
            a.linear.y = numerator_y / denominator
            a.linear.z = numerator_z / denominator

        ####################
        ## motion control ##
        ####################

        K_c = 7.0  # Constant in the formula

        dot_product = (a.linear.x * slave_pose.pose.position.x) + \
            (a.linear.y * slave_pose.pose.position.y)
        if dot_product >= 0:
            slave_vel.linear.y = (dot_product ** 1) * K_c
        else:
            slave_vel.linear.y = 0

        if math.sqrt(a.linear.x ** 2 + a.linear.y ** 2) * math.sqrt(slave_pose.pose.position.x ** 2 + slave_pose.pose.position.y ** 2) != 0:
            slave_vel.angular.z = math.acos(dot_product / (math.sqrt(a.linear.x ** 2 + a.linear.y ** 2)
                                                           * math.sqrt(slave_pose.pose.position.x ** 2 + slave_pose.pose.position.y ** 2))) * 0.5

        if (slave_pose.pose.position.z < 5):
            slave_vel.linear.z += 0.5
        pub.publish(slave_vel)
        rate.sleep()
