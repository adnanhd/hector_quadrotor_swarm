#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped


def update_position(msg, position):
    position[0] = msg.pose.position.x
    position[1] = msg.pose.position.y
    position[2] = msg.pose.position.z


def align_position(msg, args):
    velocity = args[0]
    position = args[1]
    Vcounter = args[2]
    o_des = 7.5

    # The vectors from the current agent to the rest of agents in the swarm
    o_x = msg.pose.position.x - position[0]
    o_y = msg.pose.position.y - position[1]
    o_z = msg.pose.position.z - position[2]

    if position[2] < 5.0:
        velocity[0] += 0
        velocity[1] += 0
        velocity[2] += 5 - position[2]
    elif (abs(math.sqrt(o_x * o_x + o_y * o_y + o_z * o_z) - o_des) > 0.2):
        velocity[0] += o_x - o_des
        velocity[1] += o_y - o_des
        velocity[2] += o_z - o_des
    else:
        velocity[0] = 0
        velocity[1] = 0
        velocity[2] = 0


if __name__ == '__main__':
    position = [0, 0, 0]
    msg = Twist()

    rospy.init_node('slave', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('ground_truth_to_tf/pose',
                     PoseStamped, update_position, position)

    for uav in rospy.myargv()[:]:
        rospy.Subscriber('/' + uav + '/ground_truth_to_tf/pose',
                         PoseStamped, align_position, (msg, position, pub))

    rospy.spin()
