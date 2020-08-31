#!/usr/bin/env python
"""@package swarm_control
Package applies self-organized flocking behavior for a given agent

This package is created for kovan summer internship by
@see self-organized flocking behavior
@author Adnan Harun DOGAN
@author Abulla Ahmadkhan
"""

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped, Pose
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
    slave: Pose()
            This is a variable to keep up-to-date the location and orien-
            tation information of the agent

    Returns
        nothing
    -------
    """
    # Update the slave's position as soon as the topic
    # ground_truth_to_tf/pose published
    slave.position.x = msg.pose.position.x
    slave.position.y = msg.pose.position.y
    slave.position.z = msg.pose.position.z
    slave.orientation.x = msg.pose.orientation.x
    slave.orientation.y = msg.pose.orientation.y
    slave.orientation.z = msg.pose.orientation.z
    slave.orientation.w = msg.pose.orientation.w


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
    args: ([Pose()], int)
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
    swarm_pose[agent_index].position.x = msg.pose.position.x
    swarm_pose[agent_index].position.y = msg.pose.position.y
    swarm_pose[agent_index].position.z = msg.pose.position.z
    swarm_pose[agent_index].orientation.x = msg.pose.orientation.x
    swarm_pose[agent_index].orientation.y = msg.pose.orientation.y
    swarm_pose[agent_index].orientation.z = msg.pose.orientation.z
    swarm_pose[agent_index].orientation.w = msg.pose.orientation.w


def Quad2Euler(q):
    """
    This function transforms from quernion to euler

    #Quaternion_to_Euler_Angles_Conversion
    For details, see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    Parameters
    ----------
    q : Pose().orientation
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


def level_of_distance_bwtn(a, s):

    distance = math.sqrt((a.x - s.x) ** 2 + (a.y - s.y) ** 2)

    if distance < 24:
        return int((23 - distance) / 3)
    else:
        return 0


if __name__ == '__main__':
    wps = [(8, 6, 10), (6, 8, 10), (0, 10, 10), (-6, 8, 10), (-8, 6, 10), (-10, 0, 10),
           (-8, -6, 10), (-6, -8, 10), (0, -10, 10), (6, -8, 10), (8, -6, 10), (10, 0, 10)]

    i = 0

    ## desired level of distance between any two agents
    o_des = 5       
    ## denumerator of the force in the proximal control vector
    C = 5           
    ## weight of the direction preference vector
    gamma = 10      
    ## weight of the proximal control vector
    beta = 12       
    ## Maximum forward velocity of the agent
    u_max = 2.50    
    ## Maximum angular velocity of the agent
    K_p = 1.0       

    # initialize swarm with all arguments defining swarm but excluding the agent itself
    swarm = rospy.myargv()[1:]
    
    # initialize swarm_pose with a list of Pose() objects to keep postion information
    # of the agents in the swarm up-to-date
    swarm_pose = [Pose()] * len(swarm)

    # initialize slave with a Pose() object to keep postion information of the agent 
    # up-to-date
    ## the position of the agent
    slave = Pose()
    
    # initialize msg with a Twist() object to update velocity information of the agent
    ## the velocity of the agent
    msg = Twist()

    # Initialize node to drive the agent
    rospy.init_node('agent', anonymous=True)

    # Subscribe the position of the agent itself to update the variable 'slave'
    rospy.Subscriber('ground_truth_to_tf/pose',
                     PoseStamped, slave_callback, slave)

    # Subscribe pose topic of the agents in the swarm to keep swarm_pose up-to-date
    for agent_index in range(len(swarm)):
        rospy.Subscriber('/' + swarm[agent_index] + '/ground_truth_to_tf/pose',
                         PoseStamped, agent_callback, (swarm_pose, agent_index))

    # Publish the velocity of the agent
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(100)  # 100Hz

    while not rospy.is_shutdown():
        ## the current yaw of the agent
        alpha_cur = Quad2Euler(slave.orientation)[2]
        
        '''##############################
        ## heading alignment behaviour ##
        ##############################'''

        # the swarm's cummulative (yaw) current z-axis rotation
        total_yaw = 0

        for agent in swarm_pose:
            # For each position vector `agent`, of type PoseStamped() containing position information
            # of other agents in the swarm, corresponding direction angle in xy-plane is found and ac-
            # cumulate to sine and consine of yaw value
            # the agent's current z-axis rotation (yaw)
            total_yaw += Quad2Euler(agent.orientation)[2]

        # sine and cosine components of the heading alignmnet vector
        if len(swarm_pose) != 0:
            # if there is any agent in the swarm other than the agent itself
            # calculate the cummulative heading of the swarm
            cos_head, sin_head = math.cos(
                total_yaw / len(swarm_pose)), math.sin(total_yaw / len(swarm_pose))
        else:
            # if no agent is in the swarm but the agent itself
            # the components of th HA Vector is nothing but the agents current yaw.
            cos_head, sin_head = math.cos(alpha_cur), math.sin(alpha_cur)

        '''#############################
        ## proximal control behaviour ##
        #############################'''

        cos_pose = 0
        sin_pose = 0

        for agent in swarm_pose:
            # Find distance from the difference in two axes
            o_k = level_of_distance_bwtn(agent.position, slave.position)

            # If the distance, i.e. the agent, is far from how much it is supposed to be,
            if o_k >= o_des:
                # increment force f_k by ((o_des - o_k))^2 / C
                force = -((o_des - o_k) ** 2) / C
            else:
                # otherwise, decrement by ((o_des - o_k))^2 / C.
                force = ((o_des - o_k) ** 2) / C

            disp_angle = math.atan2(
                agent.position.y - slave.position.y,  agent.position.x - slave.position.x)

            # update position vector with corresponding position vector
            cos_pose += math.cos(disp_angle) * force
            sin_pose += math.sin(disp_angle) * force

        '''##############################
        ## direction preference vector ##
        ##############################'''

        pref_yaw = math.atan2(
            wps[i][1] - slave.position.y, wps[i][0] - slave.position.x)

        delta = pref_yaw - alpha_cur

        if ((gamma != 0) and (math.sqrt((wps[i][0] - slave.position.x) ** 2 +
                                        (wps[i][1] - slave.position.y) ** 2 +
                                        (wps[i][2] - slave.position.z) ** 2) < 10)):
            i = (i+1) % len(wps)
            rospy.loginfo('changed wps to ' + str(i))

        '''#########################
        ## desired heading vector ##
        #########################'''

        # Desired Heading Vector 'a' is initialized with a Twist() object

        # initializing the numerators in 3 axes of Desired Heading Vector and the denominator
        cos_alpha = cos_head + beta * cos_pose + gamma * math.cos(delta)
        sin_alpha = sin_head + beta * sin_pose + gamma * math.sin(delta)
        alpha = math.atan2(sin_alpha, cos_alpha)

        '''#################
        ## motion control ##
        #################'''

        dot_p = math.cos(alpha) * math.cos(alpha_cur) + \
            math.sin(alpha) * math.sin(alpha_cur)

        # updating denominator with the norm of numerators
        msg.linear.x = dot_p * u_max if (dot_p > 0) else 0
        msg.linear.z = wps[i][2] - slave.position.z
        msg.angular.z = (alpha_cur - alpha) * K_p

        rospy.loginfo_throttle(
            1.0, 'alpha_des = %4.2f, alpha_cur = %4.2f' % (alpha, alpha_cur))
        rospy.loginfo_throttle(1.0, 'h = %4.2f, p = %4.2f, d = %4.2f' % (
            math.atan2(sin_head, cos_head), math.atan2(sin_pose, cos_pose), delta))
        rospy.loginfo_throttle(1.0, 'x = %4.2f, y = %4.2f, z = %4.2f' % (
            slave.position.x, slave.position.y, slave.position.z))

        pub.publish(msg)
        rate.sleep()
