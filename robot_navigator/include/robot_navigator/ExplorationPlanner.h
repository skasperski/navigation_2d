#ifndef EXPLORATION_PLANNER_H
#define EXPLORATION_PLANNER_H

#define EXPL_TARGET_SET 1
#define EXPL_FINISHED   2
#define EXPL_WAITING    3
#define EXPL_FAILED     4

#include <string>
#include <tf/transform_listener.h>
#include <nav2d_msgs/RobotPose.h>

#include "GridMap.h"

// A list of all other robots, that will subscribe to the other robots topic and update itself
typedef std::map<unsigned int, geometry_msgs::Pose2D> PoseList;

class RobotList
{
public:
	RobotList()
	{
		ros::NodeHandle robotNode;
		mOtherRobotsSubscriber = robotNode.subscribe("others", 10, &RobotList::receiveOtherPose, this);
	}
	
	void receiveOtherPose(const nav2d_msgs::RobotPose::ConstPtr& msg)
	{
		mOtherRobots[msg->robot_id] = msg->pose;
	}
	
	PoseList getRobots() { return mOtherRobots; }
	
private:
	ros::Subscriber mOtherRobotsSubscriber;
	PoseList mOtherRobots;
};

// The base class for all exploration planners
class ExplorationPlanner
{
public:
	virtual ~ExplorationPlanner() {};
	virtual int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal) = 0;
	
};

#endif
