#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "actionlib/server/simple_action_server.h"

#include <queue>

#include "GridMap.h"
#include "NavigatorCommands.h"
#include "MapInflationTool.h"
#include "ExplorationPlanner.h"
#include "robot_navigator/SendCommand.h"
#include "robot_navigator/MoveToPosition2DAction.h"
#include "robot_navigator/ExploreAction.h"
#include "robot_navigator/GetFirstMapAction.h"
#include "robot_navigator/LocalizeAction.h"

typedef actionlib::SimpleActionServer<robot_navigator::MoveToPosition2DAction> MoveActionServer;
typedef actionlib::SimpleActionServer<robot_navigator::ExploreAction> ExploreActionServer;
typedef actionlib::SimpleActionServer<robot_navigator::GetFirstMapAction> GetMapActionServer;
typedef actionlib::SimpleActionServer<robot_navigator::LocalizeAction> LocalizeActionServer;

class RobotNavigator
{
public:
	RobotNavigator();
	~RobotNavigator();

	bool receiveCommand(robot_navigator::SendCommand::Request &req, robot_navigator::SendCommand::Response &res);
	void receiveMoveGoal(const robot_navigator::MoveToPosition2DGoal::ConstPtr &goal);
	void receiveExploreGoal(const robot_navigator::ExploreGoal::ConstPtr &goal);
	void receiveGetMapGoal(const robot_navigator::GetFirstMapGoal::ConstPtr &goal);
	void receiveLocalizeGoal(const robot_navigator::LocalizeGoal::ConstPtr &goal);

private:
	bool setCurrentPosition();
	bool getMap();
	void stop();
	bool correctGoalPose();
	bool generateCommand();
	bool preparePlan();
	bool createPlan();
	void publishPlan();

	// Everything related to ROS
	tf::TransformListener mTfListener;
	ros::ServiceClient mGetMapClient;
	ros::Subscriber mGoalSubscriber;
	ros::Publisher mPlanPublisher;
	ros::Publisher mCommandPublisher;
	ros::Publisher mMarkerPublisher;
	ros::ServiceServer mCommandServer;

	std::string mMapFrame;
	std::string mRobotFrame;
	std::string mMoveActionTopic;
	std::string mExploreActionTopic;
	std::string mGetMapActionTopic;
	std::string mLocalizeActionTopic;

	MoveActionServer* mMoveActionServer;
	ExploreActionServer* mExploreActionServer;
	GetMapActionServer* mGetMapActionServer;
	LocalizeActionServer* mLocalizeActionServer;

	// Current status and goals
	bool mHasNewMap;
	bool mIsPaused;
	bool mIsStopped;
	int mStatus;
	int mRobotID;
	unsigned int mGoalPoint;
	unsigned int mStartPoint;
	double mCurrentDirection;
	double mCurrentPositionX;
	double mCurrentPositionY;

	// Everything related to the global map and plan
	MapInflationTool mInflationTool;
	std::string mExplorationStrategy;
	boost::shared_ptr<ExplorationPlanner> mExplorationPlanner;
	GridMap mCurrentMap;
	double* mCurrentPlan;

	double mInflationRadius;
	double mRobotRadius;
	unsigned int mCellInflationRadius;
	unsigned int mCellRobotRadius;

	char mCostObstacle;
	char mCostLethal;

	double mNavigationGoalDistance;
	double mNavigationGoalAngle;
	double mNavigationHomingDistance;
	double mExplorationGoalDistance;
	double mMinReplanningPeriod;
	double mMaxReplanningPeriod;
};
