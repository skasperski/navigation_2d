#ifndef MULTI_MAPPER_H
#define MULTI_MAPPER_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav2d_msgs/LocalizedScan.h"

#include <boost/thread.hpp>
#include <string>
#include <map>

#include "OpenKarto/OpenKarto.h"
#include "SelfLocalizer.h"
#include "graph_slam/SpaSolver.h"

#define ST_WAITING_FOR_MAP  10
#define ST_LOCALIZING       20
#define ST_MAPPING          30

class MultiMapper
{
public:
	// Constructor & Destructor
	MultiMapper();
	~MultiMapper();
	
	// Public methods
	void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
	void receiveLocalizedScan(const nav2d_msgs::LocalizedScan::ConstPtr& scan);
	void receiveInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
	void sendLocalizedScan(const sensor_msgs::LaserScan::ConstPtr& scan, const karto::Pose2& pose);
	void onMessage(const void* sender, karto::MapperEventArguments& args);
	bool getMap(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	void publishLoop();
	void publishTransform();
	
private:
	// Private methods
	bool updateMap();
	bool sendMap();
	void setRobotPose(double x, double y, double yaw);
	
	// Particle filter to localize within received map
	SelfLocalizer* mSelfLocalizer;
	
	// Scan-Solver
	SpaSolver* mSpaSolver;
	
	// Everything related to ROS
	tf::TransformListener mTransformListener;
	tf::TransformBroadcaster mTransformBroadcaster;
	tf::Transform mMapToOdometry;
	tf::Transform mOdometryOffset;
	
	nav_msgs::OccupancyGrid mGridMap;
	
	ros::ServiceServer mMapServer;
	ros::Publisher mMapPublisher;
	ros::Publisher mScanPublisher;
	ros::Publisher mVerticesPublisher;
	ros::Publisher mEdgesPublisher;
	ros::Publisher mPosePublisher;
	ros::Publisher mOtherRobotsPublisher;
	ros::Subscriber mLaserSubscriber;
	ros::Subscriber mScanSubscriber;
	ros::Subscriber mInitialPoseSubscriber;
	
	// Everything related to KARTO
	karto::LaserRangeFinderPtr mLaser;
	karto::SmartPointer<karto::OpenMapper> mMapper;
	std::map<int, karto::LaserRangeFinderPtr> mOtherLasers;
	bool mMapChanged;
	
	// Parameters and Variables
	int mRobotID;               // Who am I?
	double mMapResolution;      // Resolution of published grid map.
	double mRangeThreshold;     // Maximum range of laser sensor. (All robots MUST use the same due to Karto-Mapper!)
	double mMaxCovariance;      // When to accept the result of the particle filter?
	int mState;	                // What am I doing? (waiting, localizing, mapping)
	int mMapUpdateRate;	        // Publish the map every # received updates.
	bool mPublishPoseGraph;	    // Whether or not to publish the pose graph as marker-message.
	int mNodesAdded;            // Number of nodes added to the pose graph.
	int mMinMapSize;            // Minimum map size (# of nodes) needed for localization.
	ros::WallTime mLastMapUpdate;
	
	double mTransformPublishPeriod;    // Update rate for the transform map to odometry
	boost::thread* mTransformThread;
	boost::mutex mTransformMutex;
	
	// Frames and Topics
	std::string mLaserFrame;
	std::string mRobotFrame;
	std::string mOdometryFrame;
	std::string mOffsetFrame;
	std::string mMapFrame;
	std::string mLaserTopic;
	std::string mMapTopic;
	std::string mMapService;
	std::string mScanInputTopic;
	std::string mScanOutputTopic;
};

#endif
