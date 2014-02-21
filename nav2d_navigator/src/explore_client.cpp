#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <nav2d_navigator/ExploreAction.h>
#include <nav2d_navigator/SendCommand.h>

#include "commands.h"

typedef actionlib::SimpleActionClient<nav2d_navigator::ExploreAction> ExploreClient;

ExploreClient* gExploreClient;

bool receiveCommand(nav2d_navigator::SendCommand::Request &req, nav2d_navigator::SendCommand::Response &res)
{
	if(req.command == NAV_COM_EXPLORE)
	{
		nav2d_navigator::ExploreGoal goal;
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
