#include "SelfLocalizer.h"

#include <tf/transform_listener.h>
#include <math.h>

bool isNaN(double a)
{
	//return (a != a);
	return boost::math::isnan(a);
}

double angle_diff(double a, double b)
{
	a = atan2(sin(a),cos(a));
	b = atan2(sin(b),cos(b));
	double d1 = a - b;
	double d2 = 2 * M_PI - fabs(d1);
	if(d1 > 0)
	d2 *= -1.0;
	if(fabs(d1) < fabs(d2))
		return(d1);
	else
		return(d2);
}

OdometryData::OdometryData(const tf::StampedTransform& pNewPose, const tf::StampedTransform& pLastPose)
{
	mDeltaX = pNewPose.getOrigin().x() - pLastPose.getOrigin().x();
	mDeltaY = pNewPose.getOrigin().y() - pLastPose.getOrigin().y();
	mDeltaTheta = tf::getYaw(pNewPose.getRotation()) -  tf::getYaw(pLastPose.getRotation());
}

LaserData::LaserData(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	mRangeCount = scan->ranges.size();
	mRanges = new double[mRangeCount][2];
	mRangeMax = scan->range_max;
	
	double angle_min = scan->angle_min;
	double angle_inc = scan->angle_increment; 
	
	angle_inc = fmod(angle_inc + 5*M_PI, 2*M_PI) - M_PI;
	
	for(int i = 0; i < mRangeCount; i++)
	{
		if(scan->ranges[i] <= scan->range_min)
			mRanges[i][0] = scan->range_max;
		else
			mRanges[i][0] = scan->ranges[i];

		// Compute bearing
		mRanges[i][1] = angle_min + (i * angle_inc);
	}
}

LaserData::~LaserData()
{
	delete[] mRanges;
}

map_t* SelfLocalizer::sMap = NULL;
double SelfLocalizer::sSigmaHit;
double SelfLocalizer::sLamdaShort;
double SelfLocalizer::sZHit;
double SelfLocalizer::sZMax;
double SelfLocalizer::sZRand;
double SelfLocalizer::sZShort;
int SelfLocalizer::sMaxBeams;
double SelfLocalizer::sLikelihoodMaxDist;

double SelfLocalizer::sAlpha1;
double SelfLocalizer::sAlpha2;
double SelfLocalizer::sAlpha3;
double SelfLocalizer::sAlpha4;

pf_vector_t SelfLocalizer::sLaserPose;

tf::StampedTransform SelfLocalizer::mLastPose;

SelfLocalizer::SelfLocalizer(bool publish)
 : mPublishParticles(publish)
{
	mActionModelFunction = (pf_action_model_fn_t)SelfLocalizer::calculateMoveModel;
	
	ros::NodeHandle globalNode;
	globalNode.param("laser_frame", mLaserFrame, std::string("laser"));
	globalNode.param("robot_frame", mRobotFrame, std::string("robot"));
	globalNode.param("odometry_frame", mOdometryFrame, std::string("odometry_base"));
	globalNode.param("map_frame", mMapFrame, std::string("map"));
	
	ros::NodeHandle localNode("~/");
	localNode.param("min_particles", mMinParticles, 500);
	localNode.param("max_particles", mMaxParticles, 2500);
	localNode.param("alpha_slow", mAlphaSlow, 0.001);
	localNode.param("alpha_fast", mAlphaFast, 0.1);
	localNode.param("min_translation", mMinTranslation, 0.2);
	localNode.param("min_rotation", mMinRotation, 0.5);
	localNode.param("pop_err", mPopulationErr, 0.01);
	localNode.param("pop_z", mPopulationZ, 0.99);
	
	localNode.param("laser_sigma_hit", sSigmaHit, 0.2);
	localNode.param("laser_lambda_short", sLamdaShort, 0.1);
	localNode.param("laser_z_hit", sZHit, 0.95);
	localNode.param("laser_z_max", sZMax, 0.05);
	localNode.param("laser_z_rand", sZRand, 0.05);
	localNode.param("laser_z_short", sZShort, 0.1);
	localNode.param("laser_max_beams", sMaxBeams, 30);
	localNode.param("laser_likelihood_max_dist", sLikelihoodMaxDist, 2.0);
	
	localNode.param("odom_alpha_1", sAlpha1, 0.25);
	localNode.param("odom_alpha_2", sAlpha2, 0.25);
	localNode.param("odom_alpha_3", sAlpha3, 0.25);
	localNode.param("odom_alpha_4", sAlpha4, 0.25);
	
	localNode.param("laser_model_type", mLaserModelType, 2);
	switch(mLaserModelType)
	{
	case 2:
		mSensorModelFunction = (pf_sensor_model_fn_t)SelfLocalizer::calculateLikelihoodFieldModel;
		break;
	case 1:
	default:
		mLaserModelType = 1;
		mSensorModelFunction = (pf_sensor_model_fn_t)SelfLocalizer::calculateBeamModel;
	}
	
	if(mPublishParticles)
	{
		mParticlePublisher = localNode.advertise<geometry_msgs::PoseArray>("particles", 1, true);
	}

	mFirstScanReceived = false;
	mParticleFilter = NULL;
	
	// Apply tf_prefix to all used frame-id's
	mMapFrame = mTransformListener.resolve(mMapFrame);
	mOdometryFrame = mTransformListener.resolve(mOdometryFrame);
	mRobotFrame = mTransformListener.resolve(mRobotFrame);
	mLaserFrame = mTransformListener.resolve(mLaserFrame);
	
	// Use squared distance so we don't need sqrt() later
	mMinTranslation *= mMinTranslation;
	mMapToOdometry.setIdentity();
}

SelfLocalizer::~SelfLocalizer()
{
	if(mParticleFilter)
		pf_free(mParticleFilter);
	if(sMap)	
		map_free(sMap);
}

pf_vector_t SelfLocalizer::distributeParticles(void* data)
{
	map_t* map = (map_t*)data;

	double min_x, max_x, min_y, max_y;
	
	min_x = map->origin_x - (map->size_x * map->scale / 2.0);
	max_x = map->origin_x + (map->size_x * map->scale / 2.0);
	min_y = map->origin_y - (map->size_y * map->scale / 2.0);
	max_y = map->origin_y + (map->size_y * map->scale / 2.0);

	pf_vector_t p;

	while(true)
	{
		p.v[0] = min_x + drand48() * (max_x - min_x);
		p.v[1] = min_y + drand48() * (max_y - min_y);
		p.v[2] = drand48() * 2 * M_PI - M_PI;

		// Check that it's a free cell
		int i,j;
		i = MAP_GXWX(map, p.v[0]);
		j = MAP_GYWY(map, p.v[1]);
		if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
			break;
	}
	return p;
}

double SelfLocalizer::calculateMoveModel(OdometryData* data, pf_sample_set_t* set)
{
	// Implement sample_motion_odometry (Prob Rob p 136)
	double delta_rot1, delta_trans, delta_rot2;
	double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
	double delta_rot1_noise, delta_rot2_noise;

	// Avoid computing a bearing from two poses that are extremely near each
	// other (happens on in-place rotation).
	delta_trans = sqrt(data->mDeltaX * data->mDeltaX + data->mDeltaY * data->mDeltaY);
	if(delta_trans < 0.01)
		delta_rot1 = 0.0;
	else
		delta_rot1 = angle_diff(atan2(data->mDeltaY, data->mDeltaX), tf::getYaw(mLastPose.getRotation()));

	delta_rot2 = angle_diff(data->mDeltaTheta, delta_rot1);

	// We want to treat backward and forward motion symmetrically for the
	// noise model to be applied below.  The standard model seems to assume
	// forward motion.
	delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)), fabs(angle_diff(delta_rot1,M_PI)));
	delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)), fabs(angle_diff(delta_rot2,M_PI)));

	for (int i = 0; i < set->sample_count; i++)
	{
		pf_sample_t* sample = set->samples + i;

		// Sample pose differences
		delta_rot1_hat = angle_diff(delta_rot1, pf_ran_gaussian(sAlpha1*delta_rot1_noise*delta_rot1_noise + sAlpha2*delta_trans*delta_trans));
		delta_trans_hat = delta_trans - pf_ran_gaussian(sAlpha3*delta_trans*delta_trans + sAlpha4*delta_rot1_noise*delta_rot1_noise + sAlpha4*delta_rot2_noise*delta_rot2_noise);
		delta_rot2_hat = angle_diff(delta_rot2, pf_ran_gaussian(sAlpha1*delta_rot2_noise*delta_rot2_noise + sAlpha2*delta_trans*delta_trans));

		// Apply sampled update to particle pose
		sample->pose.v[0] += delta_trans_hat * cos(sample->pose.v[2] + delta_rot1_hat);
		sample->pose.v[1] += delta_trans_hat * sin(sample->pose.v[2] + delta_rot1_hat);
		sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
		sample->weight = 1.0 / set->sample_count;
	}
	
	return 0.0;
}

double SelfLocalizer::calculateBeamModel(LaserData *data, pf_sample_set_t* set)
{
	pf_sample_t *sample;
	pf_vector_t pose;
	
	double mapRange;
	double obsRange, obsBearing;
	double z, pz; // What is z/pz ?
	
	double totalWeight = 0.0;
	
	for (int j = 0; j < set->sample_count; j++)
	{
		sample = set->samples + j;
		pose = sample->pose;

		pose = pf_vector_coord_add(pose, sLaserPose);
//		pose = pf_vector_coord_sub(pose, sLaserPose);

		double p = 1.0;
		int step = (data->mRangeCount - 1) / (sMaxBeams - 1);
		for (int i = 0; i < data->mRangeCount; i+=step)
		{
			obsRange = data->mRanges[i][0];
			obsBearing = data->mRanges[i][1];
			mapRange = map_calc_range(sMap, pose.v[0], pose.v[1], pose.v[2]+obsBearing, data->mRangeMax);
			pz = 0.0;

			// Part 1: good, but noisy, hit
			z = obsRange - mapRange;
			pz += sZHit * exp(-(z * z) / (2 * sSigmaHit * sSigmaHit));

			// Part 2: short reading from unexpected obstacle (e.g., a person)
			if(z < 0)
			pz += sZShort * sLamdaShort * exp(- sLamdaShort * obsRange);

			// Part 3: Failure to detect obstacle, reported as max-range
			if(obsRange == data->mRangeMax)
			pz += sZMax * 1.0;

			// Part 4: Random measurements
			if(obsRange < data->mRangeMax)
			pz += sZRand * 1.0/data->mRangeMax;

			// TODO: outlier rejection for short readings

			assert(pz <= 1.0);
			assert(pz >= 0.0);

			p += pz*pz*pz;
		}
		sample->weight *= p;
		totalWeight += sample->weight;
	}
	return totalWeight;
}

double SelfLocalizer::calculateLikelihoodFieldModel(LaserData *data, pf_sample_set_t* set)
{
	int i, step;
	double obsRange, obsBearing;
	double totalWeight = 0.0;
	pf_sample_t* sample;
	pf_vector_t pose;
	pf_vector_t hit;

	// Compute the sample weights
	for (int j = 0; j < set->sample_count; j++)
	{
		sample = set->samples + j;
		pose = sample->pose;

		// Take into account the laser pose relative to the robot
		pose = pf_vector_coord_add(sLaserPose, pose);

		// Pre-compute a couple of things
		double zHitDenom = 2.0 * sSigmaHit * sSigmaHit;
		double zRandMult = 1.0 / data->mRangeMax;
		double p = 1.0;

		step = (data->mRangeCount - 1) / (sMaxBeams - 1);
		for (i = 0; i < data->mRangeCount; i += step)
		{
			obsRange = data->mRanges[i][0];
			obsBearing = data->mRanges[i][1];

			// This model ignores max range readings
			if(obsRange >= data->mRangeMax)
				continue;

			double z = 0.0;
			double pz = 0.0;

			// Compute the endpoint of the beam
			hit.v[0] = pose.v[0] + obsRange * cos(pose.v[2] + obsBearing);
			hit.v[1] = pose.v[1] + obsRange * sin(pose.v[2] + obsBearing);

			// Convert to map grid coords.
			int mi = MAP_GXWX(sMap, hit.v[0]);
			int mj = MAP_GYWY(sMap, hit.v[1]);

			// Part 1: Get distance from the hit to closest obstacle.
			// Off-map penalized as max distance
			if(!MAP_VALID(sMap, mi, mj))
				z = sMap->max_occ_dist;
			else
				z = sMap->cells[MAP_INDEX(sMap,mi,mj)].occ_dist;

			// Gaussian model
			// NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
			pz += sZHit * exp(-(z * z) / zHitDenom);

			// Part 2: random measurements
			pz += sZRand * zRandMult;

			// TODO: outlier rejection for short readings

			if(pz < 0.0 || pz > 1.0)
				ROS_WARN("Value pz = %.2f, but it should be in range 0..1", pz);

			// here we have an ad-hoc weighting scheme for combining beam probs
			// works well, though...
			p += (pz*pz*pz);
		}

		sample->weight *= p;
		totalWeight += sample->weight;
	}

	return(totalWeight);
}

bool SelfLocalizer::initialize()
{
	// Create the particle filter
	mParticleFilter = pf_alloc(	mMinParticles, mMaxParticles, 
								mAlphaSlow, mAlphaFast,
								SelfLocalizer::distributeParticles,
								(void*) sMap);
		
	pf_sample_set_t* set = mParticleFilter->sets + mParticleFilter->current_set;
	ROS_INFO("Initialized PF with %d samples.", set->sample_count);	
	mParticleFilter->pop_err = mPopulationErr;
	mParticleFilter->pop_z = mPopulationZ;
	
	// Distribute particles uniformly in the free space of the map
	pf_init_model(mParticleFilter, SelfLocalizer::distributeParticles, sMap);
	
	// Get the laser pose on the robot
	tf::StampedTransform laserPose;
	mTransformListener.waitForTransform(mRobotFrame, mLaserFrame, ros::Time(0), ros::Duration(5.0));
	
	try
	{
		mTransformListener.lookupTransform(mRobotFrame, mLaserFrame, ros::Time(0), laserPose);
	}
	catch(tf::TransformException e)
	{
		return false;
	}
	
	sLaserPose.v[0] = laserPose.getOrigin().getX();
	sLaserPose.v[1] = laserPose.getOrigin().getY();
	sLaserPose.v[2] = tf::getYaw(laserPose.getRotation());
	
	return true;
}

void SelfLocalizer::process(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Ignore all scans unless we got a map
	if(sMap == NULL) return;
	
	// Get the odometry pose from TF. (We use RobotFrame here instead of LaserFrame, 
	// so the motion model of the particle filter can work correctly.)
	tf::StampedTransform tfPose;
	try
	{
		mTransformListener.lookupTransform(mOdometryFrame, mRobotFrame, scan->header.stamp, tfPose);
	}
	catch(tf::TransformException e)
	{
		try
		{
			mTransformListener.lookupTransform(mOdometryFrame, mRobotFrame, ros::Time(0), tfPose);
		}
		catch(tf::TransformException e)
		{
			ROS_WARN("Failed to compute odometry pose, skipping scan (%s)", e.what());
			return;
		}
	}
	
	if(!mFirstScanReceived)
	{
		// Process the first laser scan after initialization
		mLastPose = tfPose;
		mFirstScanReceived = true;
		mProcessedScans = 0;
	}
	
	// Update motion model with odometry data
	OdometryData odom(tfPose, mLastPose);

	double dist = odom.mDeltaX * odom.mDeltaX + odom.mDeltaY * odom.mDeltaY;
	if(dist < mMinTranslation && fabs(odom.mDeltaTheta) < mMinRotation)
		return;

	mProcessedScans++;
	pf_update_action(mParticleFilter, mActionModelFunction, (void*) &odom);
	mLastPose = tfPose;

	// Update sensor model with laser data
	LaserData laser(scan);
	pf_update_sensor(mParticleFilter, mSensorModelFunction, (void*) &laser);

	// Do the resampling step
	pf_update_resample(mParticleFilter);
	
	// Create the map-to-odometry transform
	tf::Transform map2robot = getBestPose();
	tf::Stamped<tf::Pose> odom2map;
	try
	{
		tf::Stamped<tf::Pose> robot2map;
		robot2map.setData(map2robot.inverse());
		robot2map.stamp_ = scan->header.stamp;
		robot2map.frame_id_ = mRobotFrame;

		mTransformListener.transformPose(mOdometryFrame, robot2map, odom2map);
		mMapToOdometry = odom2map.inverse();
	}
	catch(tf::TransformException)
	{
		ROS_WARN("Failed to subtract base to odom transform");
	}

	// Publish the particles for visualization
	publishParticleCloud();
}

void SelfLocalizer::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
	map_t* map = map_alloc();
	ROS_ASSERT(map);

	map->size_x = map_msg.info.width;
	map->size_y = map_msg.info.height;
	map->scale = map_msg.info.resolution;
	map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
	map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

	// Convert to pf-internal format
	map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
	ROS_ASSERT(map->cells);
	for(int i = 0; i < map->size_x * map->size_y; i++)
	{
		if(map_msg.data[i] == 0)
			map->cells[i].occ_state = -1;
		else if(map_msg.data[i] == 100)
			map->cells[i].occ_state = +1;
		else
			map->cells[i].occ_state = 0;
	}

	if(sMap)	
		map_free(sMap);
		
	sMap = map;
	if(mLaserModelType == 2)
	{
		ROS_INFO("Initializing likelihood field model. This can take some time on large maps...");
		map_update_cspace(sMap, sLikelihoodMaxDist);
	}
}

double SelfLocalizer::getCovariance()
{
	pf_sample_set_t* set = mParticleFilter->sets + mParticleFilter->current_set;
	double max = set->cov.m[0][0];
	if(set->cov.m[1][1] > max) max = set->cov.m[1][1];
	if(set->cov.m[2][2] > max) max = set->cov.m[2][2];
	return max;
}

tf::Transform SelfLocalizer::getBestPose()
{
	pf_vector_t pose;
	pose.v[0] = 0;
	pose.v[1] = 0;
	pose.v[2] = 0;
	pf_sample_set_t* set = mParticleFilter->sets + mParticleFilter->current_set;
	
	// Wow, is this really the only way to do this !?!
	double max_weight = 0.0;
	
	for(int i = 0; i < set->cluster_count; i++)
	{
		double weight;
		pf_vector_t pose_mean;
		pf_matrix_t pose_cov;
		if (!pf_get_cluster_stats(mParticleFilter, i, &weight, &pose_mean, &pose_cov))
		{
			ROS_ERROR("Couldn't get stats on cluster %d", i);
			break;
		}

		if(weight > max_weight)
		{
			max_weight = weight;
			pose = pose_mean;
		}
	}

	if(max_weight > 0.0)
	{
		ROS_DEBUG("Determined pose at: %.3f %.3f %.3f", pose.v[0], pose.v[1], pose.v[2]);
	}else
	{
		ROS_ERROR("Could not get pose from particle filter!");
	}
//	pose = pf_vector_coord_sub(pose, sLaserPose);
//	return pose;
	
	tf::Transform map2robot;
	map2robot.setOrigin(tf::Vector3(pose.v[0], pose.v[1], 0));
	map2robot.setRotation(tf::createQuaternionFromYaw(pose.v[2]));

	return map2robot;
}

void SelfLocalizer::publishParticleCloud()
{
	if(!mPublishParticles) return;
	
	pf_sample_set_t* set = mParticleFilter->sets + mParticleFilter->current_set;
	
	geometry_msgs::PoseArray cloud_msg;
	cloud_msg.header.stamp = ros::Time::now();
	cloud_msg.header.frame_id = mMapFrame.c_str();
	cloud_msg.poses.resize(set->sample_count);
	for(int i = 0; i < set->sample_count; i++)
	{
		double x = set->samples[i].pose.v[0];
		double y = set->samples[i].pose.v[1];
		double yaw = set->samples[i].pose.v[2];
		tf::Pose pose(tf::createQuaternionFromYaw(yaw), tf::Vector3(x, y, 0));
		tf::poseTFToMsg(pose, cloud_msg.poses[i]);

		// Check content of the message because sometimes NaN values occur and therefore visualization in rviz breaks
		geometry_msgs::Pose pose_check = cloud_msg.poses.at(i);
		geometry_msgs::Point pt = pose_check.position;
		if(isNaN(pt.x))
			ROS_WARN("NaN occured at pt.x before publishing particle cloud...");
		if(isNaN(pt.y))
			ROS_WARN("NaN occured at pt.y before publishing particle cloud...");
		if(isNaN(pt.z))
			ROS_WARN("NaN occured at pt.z before publishing particle cloud...");
		geometry_msgs::Quaternion ori = pose_check.orientation;
		if(isNaN(ori.x)){
			ROS_WARN("NaN occured at ori.x before publishing particle cloud, setting it to zero (original x:%f y:%f yaw:%f) ...", set->samples[i].pose.v[0], set->samples[i].pose.v[1], set->samples[i].pose.v[2]);
			cloud_msg.poses.at(i).orientation.x = 0;
		}
		if(isNaN(ori.y)){
			ROS_WARN("NaN occured at ori.y before publishing particle cloud, setting it to zero (original x:%f y:%f yaw:%f) ...", set->samples[i].pose.v[0], set->samples[i].pose.v[1], set->samples[i].pose.v[2]);
			cloud_msg.poses.at(i).orientation.y = 0;
		}
		if(isNaN(ori.z))
			ROS_WARN("NaN occured at ori.z before publishing particle cloud ...");
		if(isNaN(ori.w))
			ROS_WARN("NaN occured at ori.w before publishing particle cloud ...");

	}
	mParticlePublisher.publish(cloud_msg);
}
