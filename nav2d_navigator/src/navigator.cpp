#include <ros/ros.h>

#include <nav2d_navigator/RobotNavigator.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;
	
	RobotNavigator robNav;
	
	ros::spin();
	return 0;
}
