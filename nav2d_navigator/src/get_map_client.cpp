#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <nav2d_navigator/GetFirstMapAction.h>
#include <nav2d_navigator/SendCommand.h>

#include "commands.h"

typedef actionlib::SimpleActionClient<nav2d_navigator::GetFirstMapAction> GetMapClient;

GetMapClient* gGetMapClient;

bool receiveCommand(nav2d_navigator::SendCommand::Request &req, nav2d_navigator::SendCommand::Response &res)
{
	if(req.command == NAV_COM_GETMAP)
	{
		nav2d_navigator::GetFirstMapGoal goal;
		gGetMapClient->sendGoal(goal);
		return true;
	}
	return false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "GetFirstMap");
	ros::NodeHandle n;
	
	ros::ServiceServer cmdServer = n.advertiseService(NAV_GETMAP_SERVICE, &receiveCommand);
	gGetMapClient = new GetMapClient(NAV_GETMAP_ACTION, true);
	gGetMapClient->waitForServer();
	
	ros::spin();
	
	delete gGetMapClient;
	return 0;
}
