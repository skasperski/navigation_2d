#ifndef OPERATOR_H
#define OPERATOR_H

#define NODE_NAME		"operator"
#define COMMAND_TOPIC	"cmd"
#define CONTROL_TOPIC	"cmd_vel"
#define ROUTE_TOPIC		"route"
#define PLAN_TOPIC		"desired"
#define LUT_RESOLUTION	100

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav2d_operator/cmd.h>

#include <string>

using namespace ros;
using namespace tf;
using namespace costmap_2d;

class RobotOperator
{
public:
	RobotOperator();
	~RobotOperator();
	
	void receiveCommand(const nav2d_operator::cmd::ConstPtr& msg);
	void executeCommand();
	
private:
	int calculateFreeSpace(sensor_msgs::PointCloud* cloud);
	double evaluateAction(double direction, double velocity, bool debug = false);
	double findBestDirection();
	void initTrajTable();
	
	inline sensor_msgs::PointCloud* getPointCloud(double direction, double velocity);

	Costmap2DROS* mLocalMap;
	Costmap2D* mCostmap;
	double mRasterSize;
	
	TransformListener mTfListener;
	
	Subscriber mCommandSubscriber;
	Publisher mControlPublisher;
	Publisher mTrajectoryPublisher;
	Publisher mPlanPublisher;
	Publisher mCostPublisher;
	
	double mDesiredVelocity;
	double mDesiredDirection;
	double mCurrentVelocity;
	double mCurrentDirection;
	int mDriveMode;
	
	sensor_msgs::PointCloud* mTrajTable[(LUT_RESOLUTION * 4) + 2];
	
	double mMaxVelocity;
	
	bool mPublishRoute;
	double mMaxFreeSpace;
	double mSafetyDecay;
	int mDistanceWeight;
	int mSafetyWeight;
	int mConformanceWeight;
	int mContinueWeight;

	std::string mOdometryFrame;
	std::string mRobotFrame;
	
	unsigned int mRecoverySteps;
};

#endif
