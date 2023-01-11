#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.h>
#include <nav2d_navigator/MoveToPosition2DAction.h>
#include <nav2d_navigator/ExploreAction.h>
#include <nav2d_navigator/GetFirstMapAction.h>
#include <nav2d_navigator/LocalizeAction.h>

#include <nav2d_navigator/GridMap.h>
#include <nav2d_navigator/commands.h>
#include <nav2d_navigator/MapInflationTool.h>
#include <nav2d_navigator/ExplorationPlanner.h>

#include <queue>

typedef actionlib::SimpleActionServer<nav2d_navigator::MoveToPosition2DAction> MoveActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::ExploreAction> ExploreActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::GetFirstMapAction> GetMapActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::LocalizeAction> LocalizeActionServer;
typedef pluginlib::ClassLoader<ExplorationPlanner> PlanLoader;

class RobotNavigator
{
public:
	RobotNavigator();
	~RobotNavigator();

	bool receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	void receiveMoveGoal(const nav2d_navigator::MoveToPosition2DGoal::ConstPtr &goal);
	void receiveExploreGoal(const nav2d_navigator::ExploreGoal::ConstPtr &goal);
	void receiveGetMapGoal(const nav2d_navigator::GetFirstMapGoal::ConstPtr &goal);
	void receiveLocalizeGoal(const nav2d_navigator::LocalizeGoal::ConstPtr &goal);

private:
	bool isLocalized();
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
	ros::ServiceServer mStopServer;
	ros::ServiceServer mPauseServer;

	std::string mMapFrame;
	std::string mRobotFrame;

	MoveActionServer* mMoveActionServer;
	ExploreActionServer* mExploreActionServer;
	GetMapActionServer* mGetMapActionServer;
	LocalizeActionServer* mLocalizeActionServer;

	PlanLoader* mPlanLoader;

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

	double mFrequency;
	double mInflationRadius;
	double mRobotRadius;
	unsigned int mCellInflationRadius;
	unsigned int mCellRobotRadius;

	signed char mCostObstacle;
	signed char mCostLethal;

	double mCommandTargetDistance;
	double mNavigationGoalDistance;
	double mNavigationGoalAngle;
	double mNavigationHomingDistance;
	double mExplorationGoalDistance;
	double mMinReplanningPeriod;
	double mMaxReplanningPeriod;
};
