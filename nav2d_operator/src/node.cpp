#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <nav2d_operator/RobotOperator.h>

using namespace ros;

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n("~/");

	double frequency;
	n.param("frequency", frequency, 100.0);
	ROS_INFO("Operator will run at %.2f Hz.", frequency);

	RobotOperator robOp;
	
	Rate loopRate(frequency);
	while(ok())
	{
		spinOnce();
		robOp.executeCommand();
		loopRate.sleep();
		if(loopRate.cycleTime() > ros::Duration(1.0 / frequency))
			ROS_WARN("Missed desired rate of %.2f Hz! Loop actually took %.4f seconds!",frequency, loopRate.cycleTime().toSec());
	}
	return 0;	
}
