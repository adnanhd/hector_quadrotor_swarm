#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, PoseStamped

if __name__ == '__main__':
    rospy.init_node('drone_test', anonymous=True)