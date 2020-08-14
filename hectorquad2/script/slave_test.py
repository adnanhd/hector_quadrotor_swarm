#!/usr/bin/env python

import sys
import rospy
import math
import copy
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
epsion = 0.001


def slave_callback(msg, pose):
    """
    Documentation for a function.

    More details.
    """
    # Update the agent's position as soon as the topic
    # ground_truth_to_tf/pose published
    pose = msg


def agent_callback(msg, args):
    """
    Documentation for a function.

    More details.
    """
    # initializing agents index and updating its position in the list
    swarm_pose = args[0]
    agent_index = args[1]
    swarm_pose[agent_index] = msg


def quaternionToAngle(quad):
    """
    Documentation for a function.

    More details.
    """
    # Transform the quad of the form PoseStamped().pose.orientation to angle
    # by applying atan2(y,x) => float angle from math library.
    return math.atan2(2.0*(quad.y * quad.z + quad.w * quad.x), quad.w * quad.w - quad.x * quad.x - quad.y * quad.y + quad.z * quad.z)


def TwistToStr(twist):
    return '[x:%2.1f y:%2.1f z:%2.1f][x:%2.1f y:%2.1f z:%2.1f]' % (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z)


def PoseToStr(pose):
    return '[x:%2.1f y:%2.1f z:%2.1f][x:%2.1f y:%2.1f z:%2.1f]' % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z)


if __name__ == '__main__':
    swarm = rospy.myargv()[1:]
    # TODO: ADD COMMENTS HERE
    swarm_pose = [PoseStamped()] * len(swarm)
    slave_pose = PoseStamped()
    slave_vel = Twist()
    z_yaw = 0  # not used

    # Initialize node to controll the agent
    rospy.init_node('slave', anonymous=True)

    # Subscribe the position of the agent and the rest of agents in the swarm
    rospy.Subscriber('ground_truth_to_tf/pose',
                     PoseStamped, slave_callback, slave_pose)

    for agent_index in range(len(swarm)):
        rospy.Subscriber('/' + swarm[agent_index] + '/ground_truth_to_tf/pose',
                         PoseStamped, agent_callback, (swarm_pose, agent_index))

    # Publish the velocity of the agent
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # TODO: ADD COMMENTS HERE
        heading_vector = Twist()

        #################################
        ## heading alignment behaviour ##
        #################################

        x_roll = 0  # angular motion in x-axix
        y_pitch = 0  # angular motion in y-axix

        for agent in swarm_pose:
            # TODO: proofcheck
            # the position vectors, of each agent in the swarm, of the form PoseStamped() containing position
            # information of other agents in the swarm, corresponding direction angle in xy axis is found
            # and accumulate to pitch and roll values
            angle = quaternionToAngle(agent.pose.orientation)
            x_roll += math.cos(angle)
            y_pitch += math.sin(angle)

        # The norm is the hypothenuse of the right angle of roll, pitch and angle: update expalanation
        norm = math.sqrt((x_roll * x_roll) + (y_pitch * y_pitch))

        # initialize a heading_vector
        if abs(norm) < epsion:  # epsilon is very close to zero
            heading_vector.linear.x = 0
            heading_vector.linear.y = 0
        else:
            heading_vector.linear.x = x_roll / norm
            heading_vector.linear.y = y_pitch / norm

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

        dot_product = (a.linear.x * slave_pose.pose.position.x) + \
            (a.linear.y * slave_pose.pose.position.y)
        if dot_product >= 0:
            slave_vel.linear.y = (dot_product ** 1) * 7
        else:
            slave_vel.linear.y = 0

        if math.sqrt(a.linear.x ** 2 + a.linear.y ** 2) * math.sqrt(slave_pose.pose.position.x ** 2 + slave_pose.pose.position.y ** 2) != 0:
            slave_vel.angular.z = math.acos(dot_product / (math.sqrt(a.linear.x ** 2 + a.linear.y ** 2)
                                                           * math.sqrt(slave_pose.pose.position.x ** 2 + slave_pose.pose.position.y ** 2))) * 0.5

        rospy.loginfo_throttle(4.0, 'heading ' + TwistToStr(heading_vector))
        rospy.loginfo_throttle(4.0, 'position ' + TwistToStr(position_vector))
        rospy.loginfo_throttle(4.0, 'velocity ' + TwistToStr(slave_vel))
        pub.publish(slave_vel)
        rate.sleep()
