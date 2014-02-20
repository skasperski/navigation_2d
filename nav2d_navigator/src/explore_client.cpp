#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_navigator/ExploreAction.h>
#include <robot_navigator/SendCommand.h>
#include <tf/transform_datatypes.h>
#include <robot_navigator/commands.h>

typedef actionlib::SimpleActionClient<robot_navigator::ExploreAction> ExploreClient;

ExploreClient* gExploreClient;

bool receiveCommand(robot_navigator::SendCommand::Request &req, robot_navigator::SendCommand::Response &res)
{
	if(req.command == NAV_COM_EXPLORE)
	{
		robot_navigator::ExploreGoal goal;
		gExploreClient->sendGoal(goal);
		return true;
	}
	return false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Explore");
	ros::NodeHandle n;
	
	ros::ServiceServer cmdServer = n.advertiseService(NAV_EXPLORE_SERVICE, &receiveCommand);
	gExploreClient = new ExploreClient(NAV_EXPLORE_ACTION, true);
	gExploreClient->waitForServer();
	
	ros::spin();
	
	delete gExploreClient;
	return 0;
}
