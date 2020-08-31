//! \file More elaborate test class.
/**!
 * A more elaborate class definition.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/**
 * @brief A callback function. Executed each time a new pose message arrives
 * @param msg This is of geometry_msgs/Twist message
 */
void TwistMessageReceived(const geometry_msgs::Twist &msg)
{
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "[("
										 << msg.linear.x << ","
										 << msg.linear.y << "," 
										 << msg.linear.z << "), ("
										 << msg.angular.x << ","
										 << msg.angular.y << ","
										 << msg.angular.z << ")]"
					);
}
/**!
 * 
 * 
 * @see geometry_msgs/Twist.h
 * 
 * @param argc This is the number of standard argumnets
 * @param argv This is the vector of standard argumnets 
 * 
 * @brief Reads from keyboard and sends to cmd_vel/Twist
 * 
 * @return Retruns zero unless any error is thrown
 * 
 */
int main(int argc, char **argv)
{
	// Initialize the ROS system and become a node
	ros::init(argc, argv, "hectorquad2_node");
	ros::NodeHandle nh;

	// Create a publisher object
	ros::Subscriber sub = nh.subscribe(
		"cmd_vel",
		1000,
		&TwistMessageReceived);

	// Let ROS take over.
	ros::spin();
//
	return 0;
}
