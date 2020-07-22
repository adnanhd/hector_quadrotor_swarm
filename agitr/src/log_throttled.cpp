// This program generates a single log message at each
// severity level
#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "log_throttled");
	ros::NodeHandle nh;

	while(ros::ok())
	{
		ROS_DEBUG_STREAM_THROTTLE(0.1, "This appears every 0.1 sec");
		ROS_INFO_STREAM_THROTTLE(0.3, "This appears every 0.3 sec");
		ROS_WARN_STREAM_THROTTLE(0.5, "This appears every 0.5 sec");
		ROS_ERROR_STREAM_THROTTLE(1.0, "This appears every 1 sec");
		ROS_FATAL_STREAM_THROTTLE(2.0, "This appears every 2 sec");
	}

	return 0;
}
