#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav2d_navigator/GetFirstMapAction.h>
#include <std_srvs/Trigger.h>

#include <nav2d_navigator/commands.h>

typedef actionlib::SimpleActionClient<nav2d_navigator::GetFirstMapAction> GetMapClient;

GetMapClient* gGetMapClient;

bool receiveCommand(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	nav2d_navigator::GetFirstMapGoal goal;
	gGetMapClient->sendGoal(goal);
	res.success = true;
	res.message = "Send GetFirstMapGoal to Navigator.";
	return true;
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
