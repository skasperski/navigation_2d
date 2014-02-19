#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_navigator/GetFirstMapAction.h>
#include <robot_navigator/SendCommand.h>
#include <tf/transform_datatypes.h>
#include <robot_navigator/commands.h>

typedef actionlib::SimpleActionClient<robot_navigator::GetFirstMapAction> GetMapClient;

GetMapClient* gGetMapClient;

bool receiveCommand(robot_navigator::SendCommand::Request &req, robot_navigator::SendCommand::Response &res)
{
	if(req.command == NAV_COM_GETMAP)
	{
		robot_navigator::GetFirstMapGoal goal;
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
