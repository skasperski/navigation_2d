#include "ros/ros.h"
#include "graph_slam/MultiMapper.h"

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "MultiMapper");
	ros::NodeHandle node;
	
	// Create the mapper and start main loop
	MultiMapper mapper;
	ros::spin();
	
	// Quit
	return 0;
}
