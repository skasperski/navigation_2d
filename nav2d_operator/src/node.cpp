#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "RobotOperator.h"

using namespace ros;

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n;

	RobotOperator robOp;
	
	Rate loopRate(10);
	while(ok())
	{
		robOp.executeCommand();
		spinOnce();
		loopRate.sleep();
	}
	return 0;	
}
