#include <termios.h>
#include <signal.h>
#include <iostream>
#include <stdlib.h>
#include <sys/poll.h>
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <boost/thread/thread.hpp>
#include "robot_operator/cmd.h"

#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_T 0x74
#define KEYCODE_B 0x62
#define KEYCODE_N 0x6e
#define KEYCODE_SPACE 0x20

#define COMMAND_TIMEOUT_SEC 0.2

int kfd = 0;
struct termios cooked, raw;

class TeleOperator
{
private:
	ros::NodeHandle mNode;
	ros::Publisher mCommandPublisher;
	double mSpeed;
	double mDirection;
	int mDriveMode;

public:
	TeleOperator();
	~TeleOperator() { }

	bool driveKeyboard();
	void keyboardLoop();
};


TeleOperator::TeleOperator()
{
	
	mCommandPublisher = mNode.advertise<robot_operator::cmd>("cmd", 1);
	mDirection = 0;
	mSpeed = 0;
	mDriveMode = 0;
}
	
//! Loop forever while sending drive commands based on keyboard input
bool TeleOperator::driveKeyboard()
{
	robot_operator::cmd command;

	char cmd[50];
	while(mNode.ok())
	{
		std::cin.getline(cmd, 50);
		if (cmd[0]!='w' && cmd[0]!='a' && cmd[0]!='d' && cmd[0]!='s' && cmd[0]!='e' && cmd[0]!='x')
		{
			std::cout << "unknown command:" << cmd << "\n";
			continue;
		}

		// move forward
		if (cmd[0]=='w') 
		{
			mSpeed = 1.0;
		}
		// move backward
		if (cmd[0]=='x')
		{
			mSpeed = -1.0;
		}
		// turn left 
		if (cmd[0]=='a')
		{
			mDirection -= 0.1;
		}
		// turn right 
		if (cmd[0]=='d')
		{
			mDirection += 0.1;
		}
		// stop
		if (cmd[0]=='s')
		{
			mSpeed = 0.0;
		}
		// quit
		if (cmd[0]=='e')
		{
			break;
		}

		// publish the command
		command.Turn = mSpeed;
		command.Turn = mDirection;
		command.Mode = mDriveMode;
		mCommandPublisher.publish(command);
	}
	return true;
}

int main(int argc, char** argv)
{
	// init the ROS node
	ros::init(argc,argv,"teleoperator");

	TeleOperator teleop;
	// this is for command with enter
	/* teleop.driveKeyboard();  */

	// Black magic to prevent Linux from buffering keystrokes.
	struct termios to;
	tcgetattr(STDIN_FILENO, &to);
	to.c_lflag &= ~ICANON;
	tcsetattr(STDIN_FILENO, TCSANOW, &to);

	// this is for continous command
	boost::thread t = boost::thread(boost::bind(&TeleOperator::keyboardLoop, &teleop));

	ros::spin();

	t.interrupt();
	t.join();
	tcsetattr(kfd, TCSANOW, &cooked);

	return 0;
}

void TeleOperator::keyboardLoop()
{
	char c;
	bool dirty=false;

	robot_operator::cmd command;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));

	struct pollfd ufd;
	ufd.fd = kfd;
	ufd.events = POLLIN;
	while(true)
	{
		boost::this_thread::interruption_point();
		c = std::cin.get();

		switch(c)
		{
		case KEYCODE_I:
			mSpeed = 1.0;
			dirty = true;
			break;
		case KEYCODE_K:
			mSpeed = -1.0;
			dirty = true;
			break;
		case KEYCODE_J:
			mDirection -= 0.1;
			dirty = true;
			break;
		case KEYCODE_L:
			mDirection += 0.1;
			dirty = true;
			break;
		case KEYCODE_B:
			mDriveMode = 0;
			dirty = true;
			break;
		case KEYCODE_N:
			mDriveMode = 1;
			dirty = true;
			break;
		case KEYCODE_SPACE:
			mSpeed = 0.0;
			dirty = true;
			break;
		}
		if (dirty)
		{
			if(mDirection > 1) mDirection = 1; 
			if(mDirection < -1) mDirection = -1;
			command.Mode = mDriveMode;
			command.Turn = mDirection;
			command.Velocity = mSpeed;
			mCommandPublisher.publish(command);
		}
	}
}
