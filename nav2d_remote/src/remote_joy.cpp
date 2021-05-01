#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include <nav2d_operator/cmd.h>
#include <nav2d_navigator/commands.h>


class Teleoperator
{
public:
	Teleoperator();
	
private:
	void joyCB(const sensor_msgs::Joy::ConstPtr& msg);

	ros::NodeHandle mNode;
	ros::Publisher mCommandPublisher;
	ros::Subscriber mJoySubscriber;
	ros::ServiceClient mStopClient;
	ros::ServiceClient mPauseClient;
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
			// parameters for Button and Axis 
	ros::NodeHandle remoteNode("~/");
	remoteNode.param("button_pressed", mButtonPressed, false);
	
	remoteNode.param("axis_velocity", mAxisVelocity, 4);
	remoteNode.param("axis_direction", mAxisDirection, 0);
	remoteNode.param("drive_mode", mButtonDriveMode, 5);
	remoteNode.param("pause_nav", mButtonPauseNavigator, 6);
	remoteNode.param("start_exploration", mButtonStartExploration, 0);
	remoteNode.param("button_getmap", mButtonGetMap, 3);
	remoteNode.param("stop_button", mButtonStop, 1);

}

void Teleoperator::joyCB(const sensor_msgs::Joy::ConstPtr& msg)
{
	// Ignore Button-Release events
	if(mButtonPressed)
	{
		mButtonPressed = false;
	}else
	{	
		nav2d_operator::cmd cmd;
		cmd.Turn = msg->axes[mAxisDirection] * -1.0;
		cmd.Velocity = msg->axes[mAxisVelocity];
		cmd.Mode = 0;
		if(msg->buttons[mButtonDriveMode]) cmd.Mode = 1;
		mCommandPublisher.publish(cmd);
	}

	if(msg->buttons[mButtonStop])
	{
		std_srvs::Trigger srv;
		if(!mStopClient.call(srv))
		{
			ROS_ERROR("Failed to send STOP_COMMAND to Navigator.");
		}
		return;
	}

	if(msg->buttons[mButtonPauseNavigator])
	{
		std_srvs::Trigger srv;
		if(!mPauseClient.call(srv))
		{
			ROS_ERROR("Failed to send PAUSE_COMMAND to Navigator.");
		}
		return;
	}

	if(msg->buttons[mButtonGetMap])
	{
		std_srvs::Trigger srv;
		if(!mGetMapClient.call(srv))
		{
			ROS_ERROR("Failed to send GETMAP_COMMAND to GetMap-Client.");
		}
		mButtonPressed = true;
		return;
	}

	if(msg->buttons[mButtonStartExploration])
	{
		std_srvs::Trigger srv;
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
