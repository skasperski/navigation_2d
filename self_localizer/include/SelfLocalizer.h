#ifndef SELF_LOCALIZER_H
#define SELF_LOCALIZER_H

#include "string"

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "ParticleFilter/map.h"
#include "ParticleFilter/pf.h"
#include "ParticleFilter/pf_pdf.h"

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"

class OdometryData
{
public:
	OdometryData(const tf::StampedTransform& pNewPose, const tf::StampedTransform& pLastPose);
	
	double mDeltaX;
	double mDeltaY;
	double mDeltaTheta;
};

class LaserData
{
public:
	LaserData(const sensor_msgs::LaserScan::ConstPtr& scan);
	~LaserData();
	
	int mRangeCount;
	double mRangeMax;
	double (*mRanges)[2];
};

class SelfLocalizer
{
public:
	SelfLocalizer(bool publish = true);
	~SelfLocalizer();
	
	bool initialize();
	
	// Uniformly distribute particles in the map
	static pf_vector_t distributeParticles(void* map);
	
	// Create a map from a ROS OccupancyGrid
	void convertMap(const nav_msgs::OccupancyGrid& map_msg);
	
	void process(const sensor_msgs::LaserScan::ConstPtr& scan);
	
	double getCovariance();
	tf::Transform getBestPose();

	tf::StampedTransform getMapToOdometry()
	{
		return tf::StampedTransform(mMapToOdometry, ros::Time::now(), mMapFrame, mOdometryFrame);
	}
	
	void publishParticleCloud();
	
	// These methods are static, so the particle filter can call them via callback.
	// The parameters are static, so they can be used within these two methods.
	static double calculateMoveModel(OdometryData* data, pf_sample_set_t* set);
	static double calculateBeamModel(LaserData *data, pf_sample_set_t* set);
	static double calculateLikelihoodFieldModel(LaserData *data, pf_sample_set_t* set);

private:	
	// Parameters for movement model (I don't know what they do.)
	static double sAlpha1, sAlpha2, sAlpha3, sAlpha4;
	static pf_vector_t sLaserPose;
	
	// How many range readings from every scan are used for the sensor model update
	static int sMaxBeams;
	
	// Max distance at which we care about obstacles for constructing the likelihood field
	static double sLikelihoodMaxDist;
	
	// The map
	static map_t* sMap;
	
	// Stddev of Gaussian model for laser hits
	static double sSigmaHit;
	
	// Decay rate of exponential model for short readings
	static double sLamdaShort;
	
	// Mixture params for the components of the model, must sum to 1
	static double sZHit;
	static double sZShort;
	static double sZMax;
	static double sZRand;
	
	std::string mMapFrame;
	std::string mOdometryFrame;
	std::string mRobotFrame;
	std::string mLaserFrame;
	
private:
	// The actual particle filter
	pf_t* mParticleFilter;
	int mProcessedScans;
	
	// ROS stuff
	tf::TransformListener mTransformListener;
	ros::Publisher mParticlePublisher;
	
	// The pose of the last scan used for localization
	static tf::StampedTransform mLastPose;
	
	// The current localization result
	tf::Transform mMapToOdometry;
	
	// Parameters
	int mMinParticles;
	int mMaxParticles;
	double mAlphaSlow;
	double mAlphaFast;
	double mPopulationErr;
	double mPopulationZ;	
	
	double mMinTranslation;
	double mMinRotation;
	
	bool mPublishParticles;
	bool mFirstScanReceived;
	
	int mLaserModelType; // 1 = beam, 2 = likelihood field
	
	pf_sensor_model_fn_t mSensorModelFunction;
	pf_action_model_fn_t mActionModelFunction;
};

#endif
