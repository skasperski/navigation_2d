#include <ros/ros.h>

#include <nav2d_karto/MultiMapper.h>
#include <nav2d_karto/SpaSolver.h>

#ifdef USE_G2O
#include <nav2d_karto/G2oSolver.h>
#else
#include <nav2d_karto/SpaSolver.h>
#endif

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "MultiMapper");
	ros::NodeHandle node;

	// Create a scan-solver
#ifdef USE_G2O
	G2oSolver* solver = new G2oSolver();
#else
	SpaSolver* solver = new SpaSolver();
#endif

	// Create the MultiMapper
	MultiMapper* mapper = new MultiMapper();
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
