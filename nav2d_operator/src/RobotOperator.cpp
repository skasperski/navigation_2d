#include <robot_operator/RobotOperator.h>
#include <nav_msgs/GridCells.h>

#include <math.h>

#define PI 3.14159265

RobotOperator::RobotOperator(NodeHandle* n)
{
	// Create the local costmap
	mLocalMap = new Costmap2DROS("local_map", mTfListener);
	mRasterSize = mLocalMap->getCostmap()->getResolution();
	
	// Publish / subscribe to ROS topics
	mCommandSubscriber = n->subscribe(COMMAND_TOPIC, 1, &RobotOperator::receiveCommand, this);
	mControlPublisher = n->advertise<geometry_msgs::Twist>(CONTROL_TOPIC, 1);
	mCostPublisher = n->advertise<geometry_msgs::Vector3>("costs", 1);
	
	// Get parameters from the parameter server
	NodeHandle operatorNode("~/");
	operatorNode.param("publish_route", mPublishRoute, false);
	if(mPublishRoute)
	{
		ROS_INFO("Will publish desired direction on '%s' and control direction on '%s'.", ROUTE_TOPIC, PLAN_TOPIC);
		mTrajectoryPublisher = n->advertise<nav_msgs::GridCells>(ROUTE_TOPIC, 1);
		mPlanPublisher = n->advertise<nav_msgs::GridCells>(PLAN_TOPIC, 1);
	}
	operatorNode.param("max_free_space", mMaxFreeSpace, 5.0);
	operatorNode.param("safety_decay", mSafetyDecay, 0.95);
	operatorNode.param("distance_weight", mDistanceWeight, 1);
	operatorNode.param("safety_weight", mSafetyWeight, 1);
	operatorNode.param("conformance_weight", mConformanceWeight, 1);
	operatorNode.param("continue_weight", mContinueWeight, 1);
	operatorNode.param("max_velocity", mMaxVelocity, 1.0);
	
	ros::NodeHandle robotNode;
	robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
	robotNode.param("odometry_frame", mOdometryFrame, std::string("odometry_base"));

	// Apply tf_prefix to all used frame-id's
	std::string tfPrefix = mTfListener.getTFPrefix();
	mRobotFrame = tf::resolve(tfPrefix, mRobotFrame);
	mOdometryFrame = tf::resolve(tfPrefix, mOdometryFrame);

	// Initialize the lookup table for the driving directions
	ROS_INFO("Initializing LUT...");
	initTrajTable();
	ROS_INFO("...done!");
	
	// Set internal parameters
	mDesiredDirection = 0;
	mDesiredVelocity = 0;
	mCurrentDirection = 0;
	mCurrentVelocity = 0;
	mDriveMode = 0;
	mRecoverySteps = 0;
}

RobotOperator::~RobotOperator()
{
	delete mLocalMap;
	for(int i = 0; i < LUT_RESOLUTION; i++)
	{
		delete mTrajTable[i];
	}
}

void RobotOperator::initTrajTable()
{
	for(int i = 0; i < (LUT_RESOLUTION * 4) + 2; i++)
	{
		mTrajTable[i] = NULL;
	}	
	for(int i = 1; i < LUT_RESOLUTION; i++)
	{
		double tw = -PI * i / LUT_RESOLUTION;
		double tx = cos(tw) + 1;
		double ty = -sin(tw);
		double tr = ((tx*tx)+(ty*ty))/(ty+ty);
		std::vector<geometry_msgs::Point32> points;
		double alpha = 0;
		while(alpha < PI)
		{
			double x = tr * sin(alpha);
			double y = tr * (1.0 - cos(alpha));
			geometry_msgs::Point32 p;
			p.x = x;
			p.y = y;
			p.z = 0;
			points.push_back(p);
			alpha += mRasterSize / tr;
		}
		// Add the PointCloud to the LUT
		// Circle in forward-left direction
		sensor_msgs::PointCloud* flcloud = new sensor_msgs::PointCloud();
		flcloud->header.stamp = Time(0);
		flcloud->header.frame_id = mRobotFrame;
		flcloud->points.resize(points.size());
		
		// Circle in forward-right direction
		sensor_msgs::PointCloud* frcloud = new sensor_msgs::PointCloud();
		frcloud->header.stamp = Time(0);
		frcloud->header.frame_id = mRobotFrame;
		frcloud->points.resize(points.size());
		
		// Circle in backward-left direction
		sensor_msgs::PointCloud* blcloud = new sensor_msgs::PointCloud();
		blcloud->header.stamp = Time(0);
		blcloud->header.frame_id = mRobotFrame;
		blcloud->points.resize(points.size());
		
		// Circle in backward-right direction
		sensor_msgs::PointCloud* brcloud = new sensor_msgs::PointCloud();
		brcloud->header.stamp = Time(0);
		brcloud->header.frame_id = mRobotFrame;
		brcloud->points.resize(points.size());
		
		for(unsigned int j = 0; j < points.size(); j++)
		{
			flcloud->points[j] = points[j];
			frcloud->points[j] = points[j];
			blcloud->points[j] = points[j];
			brcloud->points[j] = points[j];
			
			frcloud->points[j].y *= -1;
			blcloud->points[j].x *= -1;
			brcloud->points[j].x *= -1;
			brcloud->points[j].y *= -1;
		}
		mTrajTable[LUT_RESOLUTION - i] = flcloud;
		mTrajTable[LUT_RESOLUTION + i] = frcloud;
		mTrajTable[(3 * LUT_RESOLUTION + 1) - i] = blcloud;
		mTrajTable[(3 * LUT_RESOLUTION + 1) + i] = brcloud;
	}
	
	// Add First and Last LUT-element
	geometry_msgs::Point32 p;
	p.x = 0;
	p.y = 0;
	p.z = 0;
	
	sensor_msgs::PointCloud* turn = new sensor_msgs::PointCloud();
	turn->header.stamp = Time(0);
	turn->header.frame_id = mRobotFrame;
	turn->points.resize(1);
	turn->points[0] = p;
	
	int straight_len = 5.0 / mRasterSize;
	
	sensor_msgs::PointCloud* fscloud = new sensor_msgs::PointCloud();
	fscloud->header.stamp = Time(0);
	fscloud->header.frame_id = mRobotFrame;
	fscloud->points.resize(straight_len);
	
	sensor_msgs::PointCloud* bscloud = new sensor_msgs::PointCloud();
	bscloud->header.stamp = Time(0);
	bscloud->header.frame_id = mRobotFrame;
	bscloud->points.resize(straight_len);
	
	for(int i = 0; i < straight_len; i++)
	{
		fscloud->points[i] = p;
		bscloud->points[i] = p;
		bscloud->points[i].x *= -1;
		p.x += mRasterSize;
	}
	
	mTrajTable[LUT_RESOLUTION] = fscloud;
	mTrajTable[LUT_RESOLUTION*3 + 1] = bscloud;
	
	mTrajTable[0] = turn;
	mTrajTable[LUT_RESOLUTION*2] = turn;
	mTrajTable[LUT_RESOLUTION*2 + 1] = turn;
	mTrajTable[LUT_RESOLUTION*4 + 1] = turn;
	
	for(int i = 0; i < (LUT_RESOLUTION * 4) + 2; i++)
	{
		if(!mTrajTable[i])
		{
			ROS_ERROR("Table entry %d has not been initialized!", i);
		}
	}	
}

void RobotOperator::receiveCommand(const robot_operator::cmd::ConstPtr& msg)
{
	if(msg->Turn < -1 || msg->Turn > 1)
	{
		// The given direction is invalid.
		// Something is going wrong, so better stop the robot:
		mDesiredDirection = 0;
		mDesiredVelocity = 0;
		mCurrentDirection = 0;
		mCurrentVelocity = 0;
		ROS_ERROR("Invalid turn direction on topic '%s'!", COMMAND_TOPIC);
		return;
	}
	mDesiredDirection = msg->Turn;
	mDesiredVelocity = msg->Velocity * mMaxVelocity;
	mDriveMode = msg->Mode;
}

void RobotOperator::executeCommand()
{
	// 1. Get a copy of the costmap to work on.
	mCostmap = mLocalMap->getCostmap();
	double bestDirection, d;
	
	// 2. Set velocity and direction depending on drive mode
	switch(mDriveMode)
	{
	case 0:
//		mCurrentDirection = findBestDirection();
		bestDirection = findBestDirection();
		d = bestDirection - mCurrentDirection;
		if(d < -0.2) d = -0.2;
		if(d > 0.2) d = 0.2;
		mCurrentDirection += d;
		mCurrentVelocity = mDesiredVelocity;
		break;
	case 1:
		mCurrentDirection = mDesiredDirection;
		mCurrentVelocity = mDesiredVelocity;
		break;
	default:
		ROS_ERROR("Invalid drive mode!");
		mCurrentVelocity = 0.0;
	}
	
	// Create some Debug-Info
	evaluateAction(mCurrentDirection, mCurrentVelocity, true);
	
	sensor_msgs::PointCloud* originalCloud = getPointCloud(mCurrentDirection, mDesiredVelocity);
	sensor_msgs::PointCloud transformedCloud;

	try
	{
		mTfListener.transformPointCloud(mOdometryFrame,*originalCloud,transformedCloud);
	}
	catch(TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}
	
	// Determine maximum linear velocity
	int freeCells = calculateFreeSpace(&transformedCloud);
	double freeSpace = mRasterSize * freeCells;

	double safeVelocity = (freeSpace / mMaxFreeSpace) + 0.05;
	if(freeCells == transformedCloud.points.size() && safeVelocity < 0.5)
		safeVelocity = 0.5;
		
	if(freeSpace < 0.3 && freeCells < transformedCloud.points.size())
		safeVelocity = 0;

	if(safeVelocity > mMaxVelocity)
		safeVelocity = mMaxVelocity;

	// Check whether the robot is stuck
	if(mRecoverySteps > 0) mRecoverySteps--;
	if(safeVelocity < 0.1)
	{
		if(mDriveMode == 0)
		{
			// Turn in place (somewhat hacky)
//			mCurrentVelocity = mDesiredVelocity;
//			if(mCurrentVelocity > 0.5) mCurrentVelocity = 0.5;
//			if(mDesiredDirection > 0)
//			{
//				mCurrentDirection = 1;
//			}else
//			{
//				mCurrentDirection = -1;
//			}
			mRecoverySteps = 30; // Recover for 3 seconds
			ROS_WARN_THROTTLE(1, "Robot is stuck! Trying to recover...");
		}else
		{
			mCurrentVelocity = 0;
			ROS_WARN_THROTTLE(1, "Robot cannot move further in this direction!");
		}
	}

	// Publish route via ROS (mainly for debugging)
	if(mPublishRoute)
	{
		nav_msgs::GridCells route_msg;
		route_msg.header.frame_id = mOdometryFrame;
		route_msg.header.stamp = Time::now();
	
		route_msg.cell_width = mCostmap->getResolution();
		route_msg.cell_height = mCostmap->getResolution();
	
		route_msg.cells.resize(freeCells);
		for(int i = 0; i < freeCells; i++)
		{
			route_msg.cells[i].x = transformedCloud.points[i].x;
			route_msg.cells[i].y = transformedCloud.points[i].y;
			route_msg.cells[i].z = transformedCloud.points[i].z;
		}
		mTrajectoryPublisher.publish(route_msg);
	
		// Publish plan via ROS (mainly for debugging)
		sensor_msgs::PointCloud* originalPlanCloud = getPointCloud(mDesiredDirection, mDesiredVelocity);
		sensor_msgs::PointCloud transformedPlanCloud;

		try
		{
			mTfListener.transformPointCloud(mOdometryFrame,*originalPlanCloud,transformedPlanCloud);
		}
		catch(TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			return;
		}
		nav_msgs::GridCells plan_msg;
		plan_msg.header = route_msg.header;
	
		plan_msg.cell_width = mCostmap->getResolution();
		plan_msg.cell_height = mCostmap->getResolution();
	
		int freeSpacePlan = calculateFreeSpace(&transformedPlanCloud);
		plan_msg.cells.resize(freeSpacePlan);
		for(int i = 0; i < freeSpacePlan; i++)
		{
			plan_msg.cells[i].x = transformedPlanCloud.points[i].x;
			plan_msg.cells[i].y = transformedPlanCloud.points[i].y;
			plan_msg.cells[i].z = transformedPlanCloud.points[i].z;
		}
		mPlanPublisher.publish(plan_msg);
	}
	
	// Publish result via Twist-Message
	geometry_msgs::Twist controlMsg;
	double velocity = mCurrentVelocity;
	if(mCurrentDirection == 0)
	{
		if(velocity > safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, safeVelocity);
			velocity = safeVelocity;
		}else if(velocity < -safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, -safeVelocity);
			velocity = -safeVelocity;
		}
		controlMsg.linear.x = velocity;
		controlMsg.angular.z = 0;
	}else if(mCurrentDirection == -1 || mCurrentDirection == 1)
	{
		controlMsg.linear.x = 0;
		controlMsg.angular.z = -1.0 * mCurrentDirection * velocity;
	}else
	{
		double x = sin(mCurrentDirection * PI);
		double y = (cos(mCurrentDirection * PI) + 1);
		double r = ((x*x) + (y*y)) / (2*x);
		double abs_r = (r > 0) ? r : -r;
		velocity /= (1 + (1.0/abs_r));
		if(velocity > safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, safeVelocity);
			velocity = safeVelocity;
		}else if(velocity < -safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, -safeVelocity);
			velocity = -safeVelocity;
		}
		
		controlMsg.linear.x = velocity;
		controlMsg.angular.z = -1.0 / r * controlMsg.linear.x;
	}
	mControlPublisher.publish(controlMsg);
}

int RobotOperator::calculateFreeSpace(sensor_msgs::PointCloud* cloud)
{	
	unsigned int mx, my;
	int length = cloud->points.size();
	int freeSpace = 0;
	for(int i = 0; i < length; i++)
	{
		if(mCostmap->worldToMap(cloud->points[i].x, cloud->points[i].y, mx, my))
		{
			if(mCostmap->getCost(mx,my) < INSCRIBED_INFLATED_OBSTACLE)
			{
				freeSpace++;
			}else
			{
				break;
			}
		}else
		{
			break;
		}
	}
	return freeSpace;
}

double RobotOperator::evaluateAction(double direction, double velocity, bool debug)
{
	sensor_msgs::PointCloud* originalCloud = getPointCloud(direction, velocity);
	sensor_msgs::PointCloud transformedCloud;
	try
	{
		mTfListener.transformPointCloud(mOdometryFrame, *originalCloud,transformedCloud);
	}
	catch(TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return 1;
	}
	
	double valueDistance = 0.0;    // How far can the robot move in this direction?
	double valueSafety = 0.0;      // How safe is it to move in that direction?
	double valueConformance = 0.0; // How conform is it with the desired direction?
	
	double freeSpace = 0.0;
	double decay = 1.0;
	unsigned char cell_cost;
	double safety;
	
	// Calculate safety value
	int length = transformedCloud.points.size();
	bool gettingBetter = true;
	for(int i = 0; i < length; i++)
	{
		unsigned int mx, my;
		if(mCostmap->worldToMap(transformedCloud.points[i].x, transformedCloud.points[i].y, mx, my))
		{
			cell_cost = mCostmap->getCost(mx,my);
			if(cell_cost >= INSCRIBED_INFLATED_OBSTACLE)
			{
				// Trajectory hit an obstacle
				break;
			}
		}
		freeSpace += mRasterSize;
			
		safety = INSCRIBED_INFLATED_OBSTACLE - (cell_cost * decay);
		if(gettingBetter)
		{
			if(safety >= valueSafety) valueSafety = safety;
			else gettingBetter = false;
		}else
		{
			if(safety < valueSafety) valueSafety = safety;
		}
		decay *= mSafetyDecay;
	}
	
	double action_value = 0.0;
	double normFactor = 0.0;
	valueSafety /= INSCRIBED_INFLATED_OBSTACLE;
	
	// Calculate distance value
	if(freeSpace >= mMaxFreeSpace)
	{
		freeSpace = mMaxFreeSpace;
	}
	valueDistance = freeSpace / std::min(mMaxFreeSpace, length*mRasterSize);
	normFactor = mDistanceWeight + mSafetyWeight;

	if(mRecoverySteps == 0)
	{
		// Calculate continuety value
		double valueContinue = mCurrentDirection - direction;
		if(valueContinue < 0) valueContinue *= -1;
		valueContinue = 1.0 / (1.0 + exp(pow(valueContinue-0.5,15)));
		
		// Calculate conformance value
		double desired_sq = (mDesiredDirection > 0) ? mDesiredDirection * mDesiredDirection : mDesiredDirection * -mDesiredDirection;
		double evaluated_sq = (direction > 0) ? direction * direction : direction * -direction;
		valueConformance = cos(PI / 2.0 * (desired_sq - evaluated_sq)); // cos(-PI/2 ... +PI/2) --> [0 .. 1 .. 0]
		
		action_value += valueConformance * mConformanceWeight;
		action_value += valueContinue * mContinueWeight;
		normFactor += mConformanceWeight + mContinueWeight;
	}
	
	action_value += valueDistance * mDistanceWeight;
	action_value += valueSafety * mSafetyWeight;
	action_value /=  normFactor;
	
	if(debug)
	{
		geometry_msgs::Vector3 cost_msg;
		cost_msg.x = valueDistance;
		cost_msg.y = valueSafety;
//		cost_msg.y = valueContinue;
		cost_msg.z = valueConformance;
		mCostPublisher.publish(cost_msg); 
	}
	
	return action_value;
}

double diff(double v1, double v2)
{
	if(v1 > v2)
		return v1 - v2;
	else
		return v2 - v1;
}

double RobotOperator::findBestDirection()
{
	double best_dir = -1.0;
	double best_value = 0.0;
	double step = 0.01;
	double dir = -1.0;
	
	while(dir <= 1.0)
	{
		double value = evaluateAction(dir, mDesiredVelocity);
		if(value > best_value)
		{
			best_dir = dir;
			best_value = value;
		}
		dir += step;
	}
/*	
	geometry_msgs::Vector3 cost_msg;
	cost_msg.x = evaluateAction(mDesiredDirection, mDesiredVelocity);
	cost_msg.y = best_value;
	cost_msg.z = diff(best_dir, mDesiredDirection);
	mCostPublisher.publish(cost_msg); 
*/
	return best_dir;
}

sensor_msgs::PointCloud* RobotOperator::getPointCloud(double direction, double velocity)
{
	if(direction < -1) direction = -1;
	if(direction > 1) direction = 1;
	int offset = (velocity >= 0) ? LUT_RESOLUTION : 3*LUT_RESOLUTION + 1;
	int table_index = (direction * LUT_RESOLUTION) + offset;
	return mTrajTable[table_index];
}
