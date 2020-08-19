#!/usr/bin/env python

import sys
import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
epsion = 0.001


def slave_callback(msg, slave):
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
    # Update the slave's position as soon as the topic
    # ground_truth_to_tf/pose published
    slave.pose.position.x = msg.pose.position.x
    slave.pose.position.y = msg.pose.position.y
    slave.pose.position.z = msg.pose.position.z
    slave.pose.orientation.x = msg.pose.orientation.x
    slave.pose.orientation.y = msg.pose.orientation.y
    slave.pose.orientation.z = msg.pose.orientation.z
    slave.pose.orientation.w = msg.pose.orientation.w


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
    swarm_pose[agent_index].pose.position.x = msg.pose.position.x
    swarm_pose[agent_index].pose.position.y = msg.pose.position.y
    swarm_pose[agent_index].pose.position.z = msg.pose.position.z
    swarm_pose[agent_index].pose.orientation.x = msg.pose.orientation.x
    swarm_pose[agent_index].pose.orientation.y = msg.pose.orientation.y
    swarm_pose[agent_index].pose.orientation.z = msg.pose.orientation.z
    swarm_pose[agent_index].pose.orientation.w = msg.pose.orientation.w


def Quad2Euler(q):
    """
    This function transforms from quernion to euler

    For details, see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion

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


def level_of_distance(distance):
    if distance < 3:
        return 7
    elif distance < 50:
        return int(8 / math.log(distance))
    else:
        return 0


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

    rate = rospy.Rate(200)  # 10Hz

    while not rospy.is_shutdown():
        # Heading vector is one pillor of the swarm and here is initialized with a Twist() object
        heading_vector = Twist()

        #################################
        ## heading alignment behaviour ##
        #################################

        cos_swarm_yaw = 0  # projection of the current z-axis rotation (yaw) onto x-axix
        sin_swarm_yaw = 0  # projection of the current z-axis rotation (yaw) onto y-axix

        for agent in swarm_pose:
            # For each position vector `agent`, of type PoseStamped() containing position information
            # of other agents in the swarm, corresponding direction angle in xy-plane is found and ac-
            # cumulate to sine and consine of yaw value
            agent_yaw = Quad2Euler(agent.pose.orientation)[2] # the agent's current z-axis rotation (yaw)
            cos_swarm_yaw += math.cos(agent_yaw) # projection of the agent's current z-axis rotation (yaw) onto x-axis
            sin_swarm_yaw += math.sin(agent_yaw) # projection of the agent's current z-axis rotation (yaw) onto y-axis

        # hypotenuse of total cumulative yaw angle
        norm_swarm_yaw = math.sqrt(cos_swarm_yaw * cos_swarm_yaw + sin_swarm_yaw * sin_swarm_yaw)

        # initialize a heading_vector
        if abs(norm_swarm_yaw) < epsion:  # epsilon is very close to zero
            heading_vector.linear.x = 0
            heading_vector.linear.y = 0
        else:
            heading_vector.linear.x = cos_swarm_yaw / norm_swarm_yaw
            heading_vector.linear.y = sin_swarm_yaw / norm_swarm_yaw

        ################################
        ## proximal control behaviour ##
        ################################

        position_vector = Twist()
        o_des = 5  # Constant in the formula
        C = 5  # Constant in the formula

        for agent_pose in swarm_pose:
            # for the current and each agent in the swarm, find the difference in between and in all three axes
            delta_x = agent_pose.pose.position.x - slave_pose.pose.position.x
            delta_y = agent_pose.pose.position.y - slave_pose.pose.position.y
            delta_z = agent_pose.pose.position.z - slave_pose.pose.position.z

            # Find distance from the difference in three axes
            o_k = level_of_distance(math.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z))

            # If the distance, i.e. the agent, is far from how much it is supposed to be,
            if o_k >= o_des:
                # increment force f_k by ((o_des - o_k))^2 / C
                force = -((o_des - o_k) ** 2) / C
            else:
                # otherwise, decrement by ((o_des - o_k))^2 / C.
                force = ((o_des - o_k) ** 2) / C

            agent_yaw = math.atan2(delta_y, delta_x)

            # update position vector with corresponding position vector
            position_vector.linear.x += math.cos(agent_yaw) * force
            position_vector.linear.y += math.sin(agent_yaw) * force

        ############################
        ## desired heading vector ##
        ############################

        # Desired Heading Vector 'a' is initialized with a Twist() object
        alpha = Twist()
        beta = 12  # Constant in the formula

        # initializing the numerators in 3 axes of Desired Heading Vector and the denominator
        cos_alpha = heading_vector.linear.x + beta * position_vector.linear.x
        sin_alpha = heading_vector.linear.y + beta * position_vector.linear.y
        nor_alpha = math.sqrt(cos_alpha * cos_alpha + sin_alpha * sin_alpha)

        alpha.linear.x = cos_alpha / nor_alpha
        alpha.linear.y = sin_alpha / nor_alpha

        ####################
        ## motion control ##
        ####################
        
        # updating numerators with heading vector and proximal control vector in corresponding coodinate axis
        u_max = 2.75

        slave_yaw = Quad2Euler(slave_pose.pose.orientation)[2]

        dot_product = alpha.linear.x * math.cos(slave_yaw) + alpha.linear.y * math.sin(slave_yaw)

        # updating denominator with the norm of numerators
        
        K_p = 1.5  # Constant in the formula

        slave_vel.linear.x = dot_product * u_max if (dot_product > 0) else 0
        slave_vel.linear.z = swarm_pose[0].pose.position.z - slave_pose.pose.position.z
        slave_vel.angular.z = (math.atan2(sin_alpha, cos_alpha) -  slave_yaw) * K_p


        pub.publish(slave_vel)
        rate.sleep()
