#include <ros/ros.h>

#include "MultiMapper.h"
#include "SpaSolver.h"

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "MultiMapper");
	ros::NodeHandle node;

	// Create the mapper and scan-solver
	MultiMapper* mapper = new MultiMapper();
	SpaSolver* solver = new SpaSolver();
	mapper->setScanSolver(solver);

	// Start main loop
	ros::Rate publishRate(10);
	while(ros::ok())
	{
		mapper->publishTransform();
		ros::spinOnce();
		publishRate.sleep();
	}

	// Quit
	delete mapper;
	delete solver;
	return 0;
}
