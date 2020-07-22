// This program generates a single log message at each
// severity level
#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "log_once");
	ros::NodeHandle nh;

	while(ros::ok())
	{
		ROS_DEBUG_STREAM_ONCE("This appears only once");
		ROS_INFO_STREAM_ONCE("This appears only once");
		ROS_WARN_STREAM_ONCE("This appears only once");
		ROS_ERROR_STREAM_ONCE("This appears only once");
		ROS_FATAL_STREAM_ONCE("This appears only once");
	}

	return 0;
}
