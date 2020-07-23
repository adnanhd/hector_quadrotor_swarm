#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hectorquad2_node");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Driver connected...");
	return 0;
}
