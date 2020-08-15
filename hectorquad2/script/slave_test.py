#!/usr/bin/env python
import sys
import rospy
import math
import copy
from geometry_msgs.msg import Twist, Quaternion, PoseStamped

allignment_data = []
msg_twist = [0, 0, 0, 1.0]
msg_pose = PoseStamped()

def heading_allignment2(loc):
	global allignment_data
	#rospy.loginfo(str(loc.pose.position.x))
	flag = False
	for data in allignment_data:
		if data[0] == 2:
			data[1] = loc
			flag = True
	if flag == False:
		allignment_data.append([2, loc])
'''
def heading_allignment3(loc):
	global allignment_data
	flag = False
	for data in allignment_data:
		if data[0] == 3:
			data[1] = loc
			flag = True
	if flag == False:
		allignment_data.append([3, loc])

def heading_allignment4(loc):
	global allignment_data
	flag = False
	for data in allignment_data:
		if data[0] == 4:
			data[1] = loc
			flag = True
	if flag == False:
		allignment_data.append([4, loc])

def heading_allignment5(loc):
	global allignment_data
	flag = False
	for data in allignment_data:
		if data[0] == 5:
			data[1] = loc
			flag = True
	if flag == False:
		allignment_data.append([5, loc])'''

def focal_control(data):
	global msg_pose
	msg_pose = data

if __name__ == '__main__':

	rospy.init_node('driver', anonymous=True)	
	node_name = rospy.myargv()[1]

	rospy.Subscriber('ground_truth_to_tf/pose', PoseStamped, focal_control)

	rospy.Subscriber('/' + rospy.myargv()[1] + '/ground_truth_to_tf/pose', PoseStamped, heading_allignment2)


	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	rate = rospy.Rate(5)

	while not rospy.is_shutdown():
		msg = Twist()
		heading_vector = Twist()

		if(len(allignment_data) != 0 and abs(allignment_data[0][1].pose.position.z - msg_pose.pose.position.z) > 1):
			#rospy.loginfo(allignment_data[0][1].pose.position.z)
			msg.linear.z = allignment_data[0][1].pose.position.z - msg_pose.pose.position.z
			pub.publish(msg)
			rate.sleep()
			continue

		# heading alignment behaviour
		numerator_x = 0
		numerator_y = 0
		denominator = 0
		
		for data in allignment_data:
			q = data[1].pose.orientation
			yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - (2 * (q.y * q.y + q.z * q.z)))
			#rospy.loginfo(str(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - (2 * (q.y * q.y + q.z * q.z)))))
			numerator_x += math.cos(yaw)
			numerator_y += math.sin(yaw)

		denominator = math.sqrt((numerator_x * numerator_x) + (numerator_y * numerator_y))
		if denominator == 0:
			denominator = 1
		heading_vector.linear.x = numerator_x / denominator
		heading_vector.linear.y = numerator_y / denominator

		#rospy.loginfo(str(heading_vector.linear.x) + " " + str(heading_vector.linear.y))

		#proximal control behaviour
		force = 0
		position_vector = Twist()
		for data in allignment_data:
			distance = math.sqrt((data[1].pose.position.x - msg_pose.pose.position.x) * (data[1].pose.position.x - msg_pose.pose.position.x) + 
				(data[1].pose.position.y - msg_pose.pose.position.y) * (data[1].pose.position.y - msg_pose.pose.position.y))
			if distance > 2:
				force = (2 * (2 - distance)) * (2 * (2 - distance)) / 5
				position_vector.linear.x += msg_pose.pose.position.x * force
				position_vector.linear.y += msg_pose.pose.position.y * force
			else:
				force = - (2 * (2 - distance)) * (2 * (2 - distance)) / 5
				position_vector.linear.x += msg_pose.pose.position.x * force
				position_vector.linear.y += msg_pose.pose.position.y * force
			rospy.loginfo(str(force) + " " + str(distance))
		#rospy.loginfo(str(position_vector.linear.x) + " " + str(position_vector.linear.y))

		#desired heading vector
		a = Twist()
		numerator_x = 0
		numerator_y = 0
		denominator = 0

		numerator_x = heading_vector.linear.x + (12 * position_vector.linear.x)
		numerator_y = heading_vector.linear.y + (12 * position_vector.linear.y)

		denominator = math.sqrt((numerator_x * numerator_x) + (numerator_y * numerator_y))
		if denominator == 0:
			denominator = 1

		a.linear.x = numerator_x / denominator
		a.linear.y = numerator_y / denominator

		#motion control
		q = msg_pose.pose.orientation
		yaw_current = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - (2 * (q.y * q.y + q.z * q.z)))
		
		dot_product = (a.linear.x * math.cos(yaw_current)) + (a.linear.y * math.sin(yaw_current))
		if dot_product >= 0:
			msg.linear.x = (dot_product ** 1) * 7
		else:
			msg.linear.x = 0
		if math.sqrt(a.linear.x ** 2 + a.linear.y ** 2) * math.sqrt(math.cos(yaw_current) ** 2 + math.sin(yaw_current) ** 2) != 0:
			if((a.linear.x < 0 and a.linear.y < 0) or (a.linear.x > 0 and a.linear.y < 0)):
				msg.angular.z = - math.acos(dot_product / (math.sqrt(a.linear.x ** 2 + a.linear.y ** 2) * math.sqrt(math.cos(yaw_current) ** 2 + math.sin(yaw_current) ** 2))) * 0.5
			else:
				msg.angular.z = math.acos(dot_product / (math.sqrt(a.linear.x ** 2 + a.linear.y ** 2) * math.sqrt(math.cos(yaw_current) ** 2 + math.sin(yaw_current) ** 2))) * 0.5
		#rospy.loginfo(str(math.acos(dot_product / (math.sqrt(a.linear.x ** 2 + a.linear.y ** 2) * math.sqrt(msg_pose.pose.position.x ** 2 + msg_pose.pose.position.y ** 2)))) + "!")
		#rospy.loginfo(str(a))
		#rospy.loginfo((str(msg.angular.z)))
		#rospy.loginfo(str(math.cos(yaw_current)) + " " + str(math.sin(yaw_current)))
		#rospy.loginfo(str(msg.linear.x) + " " + str(msg.linear.y) + " " + str(msg.linear.z) + " " + str(msg.angular.x) + " " + str(msg.angular.y) + " " + str(msg.angular.z))  
		pub.publish(msg)
		rate.sleep()