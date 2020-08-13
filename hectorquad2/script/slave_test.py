
#!/usr/bin/env python
""" 
 @package controller node for slave agents in the swarm

 it is written by Adnan Harun Dogan and Abdulla Ahmadkhan
"""

import sys
import rospy
import math
import copy
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
epsion = 0.001


def focal_control(msg, pose):
    """
    Documentation for a function.

    More details.
    """
    # Update the agent's position as soon as the topic
    # ground_truth_to_tf/pose published
    pose = msg


def slave_control(msg, args):
    """
    Documentation for a function.

    More details.
    """
    # initializing agents index and updating its position in the list
    agent_index = args[0]
    swarm = args[1]
    swarm[agent_index] = msg


def quaternionToAngle(quad):
    """
    Documentation for a function.

    More details.
    """
    # Transform the quad of the form PoseStamped().pose.orientation to angle
    # by applying atan2(y,x) => float angle from math library.
    return math.atan2(2.0*(quad.y * quad.z + quad.w * quad.x), quad.w * quad.w - quad.x * quad.x - quad.y * quad.y + quad.z * quad.z)


if __name__ == '__main__':
    # TODO: ADD COMMENTS HERE
    swarm = [PoseStamped()] * len(rospy.myargv()[1:])
    agent_vel = [0, 0, 0, 1.0]
    agent = PoseStamped()
    z_yaw = 0

    # Initialize node to controll the agent
    rospy.init_node('slave', anonymous=True)

    # Subscribe the position of the agent and the rest of agents in the swarm
    rospy.Subscriber('ground_truth_to_tf/pose',
                     PoseStamped, focal_control)

    for agent in rospy.myargv()[1:]:
        rospy.Subscriber('/' + agent + '/ground_truth_to_tf/pose',
                         PoseStamped, slave_control)

    # Publish the velocity of the agent
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # TODO: ADD COMMENTS HERE
        heading_vector = Twist()
        msg = Twist()

        #################################
        ## heading alignment behaviour ##
        #################################

        x_roll = 0  # angular motion in x-axix
        y_pitch = 0  # angular motion in y-axix

        for agent in swarm:
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

        for other in swarm:
            # for the current and each agent in the swarm, find the difference in between and in all three axes
            d_x = other.pose.position.x - agent.pose.position.x
            d_y = other.pose.position.y - agent.pose.position.y
            d_z = other.pose.position.z - agent.pose.position.z

            # Find distance from the difference in three axes
            distance = math.sqrt(d_x * d_x + d_y * d_y + d_z * d_z)

            # If the distance, i.e. the agent, is far from how much it is supposed to be,
            if distance > o_des:
                # take force f_k to be ((o_des - o_k))^2 / C
                force = ((o_des - distance) ** 2) / C
            else:
                # otherwise, add a minus sign before the right hand side.
                force = - ((o_des - distance) ** 2) / C

        # update position vector with corresponding position vector
        position_vector.linear.x = agent.pose.position.x * force
        position_vector.linear.y = agent.pose.position.y * force
        position_vector.linear.z = agent.pose.position.z * force

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

        dot_product = (a.linear.x * agent.pose.position.x) + \
            (a.linear.y * agent.pose.position.y)
        if dot_product >= 0:
            msg.linear.y = (dot_product ** 1) * 7
        else:
            msg.linear.y = 0

        if math.sqrt(a.linear.x ** 2 + a.linear.y ** 2) * math.sqrt(agent.pose.position.x ** 2 + agent.pose.position.y ** 2) != 0:
            msg.angular.z = math.acos(dot_product / (math.sqrt(a.linear.x ** 2 + a.linear.y ** 2)
                                                     * math.sqrt(agent.pose.position.x ** 2 + agent.pose.position.y ** 2))) * 0.5

        rospy.loginfo(str(msg.linear.x) + " " + str(msg.linear.y) + " " + str(msg.linear.z) +
                      " " + str(msg.angular.x) + " " + str(msg.angular.y) + " " + str(msg.angular.z))
        pub.publish(msg)
        rate.sleep()
