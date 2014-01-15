#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "robot_operator/cmd.h"
#include "robot_navigator/SendCommand.h"
#include "NavigatorCommands.h"

/******************************************************
Buttons:
 0: A
 1: B
 2: X
 3: Y
 4: LB
 5: RB
 6: BACK
 7: START
 8: Logitech*
 9: Left Stick
10: Right Stick

 ******************************************************/

class Teleoperator
{
public:
	Teleoperator();
	
private:
	void joyCB(const sensor_msgs::Joy::ConstPtr& msg);

	ros::NodeHandle mNode;
	ros::Publisher mCommandPublisher;
	ros::Subscriber mJoySubscriber;
	ros::ServiceClient mNavigatorClient;
	ros::ServiceClient mExploreClient;
	ros::ServiceClient mGetMapClient;
	
	int mAxisVelocity;
	int mAxisDirection;
	int mButtonDriveMode;
	int mButtonPauseNavigator;
	int mButtonStartExploration;
	int mButtonGetMap;
	int mButtonStop;
	
	bool mButtonPressed;
};

Teleoperator::Teleoperator()
{
	// Button and Axis configuration
	mAxisVelocity = 4;
	mAxisDirection = 0;

	mButtonDriveMode = 5;
	mButtonPauseNavigator = 6;
	mButtonStartExploration = 0;
	mButtonGetMap = 3;
	mButtonStop = 1;
	
	mCommandPublisher = mNode.advertise<robot_operator::cmd>("cmd", 1);
	mJoySubscriber = mNode.subscribe<sensor_msgs::Joy>("joy", 10, &Teleoperator::joyCB, this);
	mNavigatorClient = mNode.serviceClient<robot_navigator::SendCommand>(NAV_COMMAND_SERVICE);
	mExploreClient = mNode.serviceClient<robot_navigator::SendCommand>(NAV_EXPLORE_SERVICE);
	mGetMapClient = mNode.serviceClient<robot_navigator::SendCommand>(NAV_GETMAP_SERVICE);
	
	mButtonPressed = false;
}

void Teleoperator::joyCB(const sensor_msgs::Joy::ConstPtr& msg)
{
	// Ignore Button-Release events
	if(mButtonPressed)
	{
		mButtonPressed = false;
	}else
	{	
		robot_operator::cmd cmd;
		cmd.Turn = msg->axes[mAxisDirection] * -1.0;
		cmd.Velocity = msg->axes[mAxisVelocity];
		cmd.Mode = 0;
		if(msg->buttons[mButtonDriveMode]) cmd.Mode = 1;
		mCommandPublisher.publish(cmd);
	}

	if(msg->buttons[mButtonStop])
	{
		robot_navigator::SendCommand srv;
		srv.request.command = NAV_COM_STOP;
		if(!mNavigatorClient.call(srv))
		{
			ROS_ERROR("Failed to send STOP_COMMAND to Navigator.");
		}
		return;
	}

	if(msg->buttons[mButtonPauseNavigator])
	{
		robot_navigator::SendCommand srv;
		srv.request.command = NAV_COM_PAUSE;
		if(!mNavigatorClient.call(srv))
		{
			ROS_ERROR("Failed to send PAUSE_COMMAND to Navigator.");
		}
		return;
	}

	if(msg->buttons[mButtonGetMap])
	{
		robot_navigator::SendCommand srv;
		srv.request.command = NAV_COM_GETMAP;
		if(!mGetMapClient.call(srv))
		{
			ROS_ERROR("Failed to send GETMAP_COMMAND to GetMap-Client.");
		}
		mButtonPressed = true;
		return;
	}

	if(msg->buttons[mButtonStartExploration])
	{
		robot_navigator::SendCommand srv;
		srv.request.command = NAV_COM_EXPLORE;
		if(!mExploreClient.call(srv))
		{
			ROS_ERROR("Failed to send EXPLORE_COMMAND to Explore-Client.");
		}
		mButtonPressed = true;
		return;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleoperator");
	Teleoperator tele_op;
	
	ros::spin();
}
