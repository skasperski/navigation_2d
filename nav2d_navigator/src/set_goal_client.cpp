#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav2d_navigator/MoveToPosition2DAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include "commands.h"

typedef actionlib::SimpleActionClient<nav2d_navigator::MoveToPosition2DAction> MoveClient;

MoveClient* gMoveClient;

void receiveGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	nav2d_navigator::MoveToPosition2DGoal goal;
	goal.target_pose.x = msg->pose.position.x;
	goal.target_pose.y = msg->pose.position.y;
	goal.target_pose.theta = tf::getYaw(msg->pose.orientation);
	goal.target_distance = 0.25;
	goal.target_angle = 0.1;
	
	gMoveClient->sendGoal(goal);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SetGoal");
	ros::NodeHandle n;
	
	ros::Subscriber goalSubscriber = n.subscribe("goal", 1, &receiveGoal);
	gMoveClient = new MoveClient(NAV_MOVE_ACTION, true);
	gMoveClient->waitForServer();
	
	ros::spin();
	
	delete gMoveClient;
	return 0;
}
