#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

#include <string.h>
#include <vector>

#define COMMAND_TIMEOUT_SEC 0.5

const int PUBLISH_FREQ = 20;

class SimOperator
{
public:
	SimOperator(unsigned int numOfRobots);

	void receiveJoyMessage(const sensor_msgs::Joy::ConstPtr& msg);
	void sendControl();

private:
	unsigned int mNumberOfRobots;
	double mMaxLinearVelocity;
	double mMaxAngularVelocity;

	ros::Subscriber mJoySubscriber;
	std::vector<ros::Publisher> mCommandPublisher;
	std::vector<geometry_msgs::Twist> mCommand;
};

SimOperator::SimOperator(unsigned int numOfRobots)
{
	mNumberOfRobots = numOfRobots;
	mMaxLinearVelocity = 1.0;
	mMaxAngularVelocity = 1.0;

	ros::NodeHandle node;	

	// Subscribe to joystick
	mJoySubscriber = node.subscribe("joy", 10, &SimOperator::receiveJoyMessage, this);

	// Publish commands for every robot in the simulation
	char topic[50];
	for(unsigned int i = 0; i < mNumberOfRobots; i++)
	{
		if(mNumberOfRobots > 1)
			sprintf(topic, "/robot_%d/cmd_vel", i);
		else
			sprintf(topic, "/cmd_vel");
		mCommandPublisher.push_back(node.advertise<geometry_msgs::Twist>(topic, 1));

		geometry_msgs::Twist twist;
		mCommand.push_back(twist);
	}
}

void SimOperator::receiveJoyMessage(const sensor_msgs::Joy::ConstPtr& msg)
{
	for(unsigned int i = 0; i < mNumberOfRobots; i++)
	{
		if(msg->buttons[i])
		{
			mCommand[i].linear.x = msg->axes[1] * mMaxLinearVelocity;
			mCommand[i].angular.z = msg->axes[0] * mMaxAngularVelocity;
		}else
		{
			mCommand[i].linear.x = 0;
			mCommand[i].angular.z = 0;
		}
	}
}

void SimOperator::sendControl()
{
	for(unsigned int i = 0; i < mNumberOfRobots; i++)
	{
		mCommandPublisher[i].publish(mCommand[i]);
	}
}

int main(int argc, char** argv)
{
	// init the ROS node
	ros::init(argc,argv,"SimController");
	ros::NodeHandle node;

	unsigned int num = 1;
	if(argc > 1)
	{
		num = (unsigned int)atoi(argv[1]);
		if(num > 6) num = 6;
	}
	
	ROS_INFO("Using Joystick to operate %d robots in simulation!", num);
	SimOperator simOp(num);

	ros::Rate pub_rate(PUBLISH_FREQ);
	while (node.ok())
	{
		ros::spinOnce();
		simOp.sendControl();
		pub_rate.sleep();
	}

	return 0;
}

