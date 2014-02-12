#include "ros/ros.h"
#include "graph_slam/MultiMapper.h"
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
	ros::spin();
	
	// Quit
	delete mapper;
	delete solver;
	return 0;
}
