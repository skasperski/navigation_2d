// MinPos Exploration Planner
// Detailed description in:
// Bautin, A., & Simonin, O. (2012). MinPos : A Novel Frontier Allocation Algorithm for Multi-robot Exploration. ICIRA, 496–508.

#include "MinPosPlanner.h"
#include <visualization_msgs/Marker.h>

MinPosPlanner::MinPosPlanner()
{
	ros::NodeHandle robotNode;
	robotNode.param("robot_id", mRobotID, 1);
	
	ros::NodeHandle navigatorNode("~/");
	navigatorNode.param("min_target_area_size", mMinTargetAreaSize, 10.0);
	navigatorNode.param("visualize_frontiers", mVisualizeFrontiers, false);
	
	if(mVisualizeFrontiers)
	{
		mFrontierPublisher = navigatorNode.advertise<visualization_msgs::Marker>("frontiers", 1, true);
	}
	
	mPlan = NULL;
}

MinPosPlanner::~MinPosPlanner()
{
	if(mPlan)
		delete[] mPlan;
}

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

int MinPosPlanner::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{
	// Create some workspace for the wavefront algorithm
	unsigned int mapSize = map->getSize();
	if(mPlan)
		delete[] mPlan;
	mPlan = new double[mapSize];
	for(unsigned int i = 0; i < mapSize; i++)
	{
		mPlan[i] = -1;
	}

	mOffset[0] = -1;					// left
	mOffset[1] =  1;					// right
	mOffset[2] = -map->getWidth();	// up
	mOffset[3] =  map->getWidth();	// down
	mOffset[4] = -map->getWidth() - 1;
	mOffset[5] = -map->getWidth() + 1;
	mOffset[6] =  map->getWidth() - 1;
	mOffset[7] =  map->getWidth() + 1;
	
	// 1. Frontiers identification and clustering
	// =========================================================================
	mFrontiers.clear();
	mFrontierCells = 0;

	// Initialize the queue with the robot position
	Queue queue;
	Entry startPoint(0.0, start);
	queue.insert(startPoint);
	mPlan[start] = 0;
	
	Queue::iterator next;
	unsigned int x, y, index;
	double linear = map->getResolution();
	int cellCount = 0;

	// Search for frontiers with wavefront propagation
	while(!queue.empty())
	{
		cellCount++;
		
		// Get the nearest cell from the queue
		next = queue.begin();
		double distance = next->first;
		index = next->second;
		queue.erase(next);
		
		// Now continue 1st level WPA
		for(unsigned int it = 0; it < 4; it++)
		{
			unsigned int i = index + mOffset[it];
			if(mPlan[i] == -1 && map->isFree(i))
			{
				// Check if it is a frontier cell
				if(map->isFrontier(i))
				{
					findCluster(map, i);
				}else
				{
					queue.insert(Entry(distance+linear, i));
				}
				mPlan[i] = distance+linear;
			}
		}
	}

	ROS_DEBUG("[MinPos] Found %d frontier cells in %d frontiers.", mFrontierCells, (int)mFrontiers.size());
	if(mFrontiers.size() == 0)
	{
		if(cellCount > 50)
		{
			return EXPL_FINISHED;
		}else
		{
			ROS_WARN("[MinPos] No Frontiers found after checking %d cells!", cellCount);
			return EXPL_FAILED;
		}
	}
	
	// Publish frontiers as marker for RVIZ
	if(mVisualizeFrontiers)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CUBE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = map->getOriginX() + (map->getResolution() / 2);
		marker.pose.position.y = map->getOriginY() + (map->getResolution() / 2);
		marker.pose.position.z = map->getResolution() / 2;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = map->getResolution();
		marker.scale.y = map->getResolution();
		marker.scale.z = map->getResolution();
		marker.color.a = 0.5;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.points.resize(mFrontierCells);
		marker.colors.resize(mFrontierCells);
		
		unsigned int p = 0;
		srand(1337);
		for(unsigned int i = 0; i < mFrontiers.size(); i++)
		{
			char r = rand() % 256;
			char g = rand() % 256;
			char b = rand() % 256;
			for(unsigned int j = 0; j < mFrontiers[i].size(); j++)
			{
				if(p < mFrontierCells)
				{
					if(!map->getCoordinates(x, y, mFrontiers[i][j]))
					{
						ROS_ERROR("[MinPos] getCoordinates failed!");
						break;
					}
					marker.points[p].x = x * map->getResolution();
					marker.points[p].y = y * map->getResolution();
					marker.points[p].z = 0;
					
					marker.colors[p].r = r;
					marker.colors[p].g = g;
					marker.colors[p].b = b;
					marker.colors[p].a = 0.5;
				}else
				{
					ROS_ERROR("[MinPos] SecurityCheck failed! (Asked for %d / %d)", p, mFrontierCells);
				}
				p++;
			}
		}
		mFrontierPublisher.publish(marker);
	}
	
	// 2. Computation of the distances to frontiers
	unsigned int bestRank = 9999;
	unsigned int bestFrontier = 0;
	double bestDistance = 9999;
	
	// Get positions of other robots
	std::set<unsigned int> indices;
	PoseList l = mRobotList.getRobots();
	for(PoseList::iterator p = l.begin(); p != l.end(); p++)
	{
		if((int)p->first == mRobotID) continue;
		unsigned int robot_x =  (double)(p->second.x - map->getOriginX()) / map->getResolution();
		unsigned int robot_y =  (double)(p->second.y - map->getOriginY()) / map->getResolution();
		unsigned int robot = 0;
		if(map->getIndex(robot_x, robot_y, robot))
		{
			indices.insert(robot);
			ROS_DEBUG("[MinPos] Inserted robot at index %d.", robot);
		}else
		{
			ROS_WARN("[MinPos] Couldn't get index of robot %d!", p->first);
		}
	}
	ROS_DEBUG("[MinPos] Using known positions of %d other robots.", (int)indices.size());
	
	for(unsigned int frontier = 0; frontier < mFrontiers.size(); frontier++)
	{
		// Reset the plan
		double* plan = new double[mapSize];
		for(unsigned int i = 0; i < mapSize; i++) plan[i] = -1;
		Queue frontQueue;
		unsigned int rank = 0;
		double distanceToRobot = -1;
		
		// Add all cells of current frontier
		for(unsigned int cell = 0; cell < mFrontiers[frontier].size(); cell++)
		{
			plan[mFrontiers[frontier][cell]] = 0;
			frontQueue.insert(Entry(0.0, mFrontiers[frontier][cell]));
		}
		
		// Start wavefront propagation
		while(!frontQueue.empty())
		{
			// Get the nearest cell from the queue
			Queue::iterator next = frontQueue.begin();
			double distance = next->first;
			unsigned int index = next->second;
			frontQueue.erase(next);
			
			if(index == start) // Wavefront reached start point
			{
				distanceToRobot = distance;
				break;
			}
			if(indices.find(index) != indices.end()) rank++; 
			
			for(unsigned int it = 0; it < 4; it++)
			{
				int ind = index + mOffset[it];
				if(ind >= 0 && ind < (int)map->getSize() && map->getData(ind) == 0 && plan[ind] == -1)
				{
					plan[ind] = distance + map->getResolution();
					frontQueue.insert(Entry(distance + map->getResolution(), ind));
				}
			}
		}
		ROS_DEBUG("[MinPos] Frontier %d has rank %d and distance %.2f.", frontier, rank, distanceToRobot);
		if(distanceToRobot > 0 && (rank < bestRank || (rank == bestRank && distanceToRobot < bestDistance)))
		{
			bestRank = rank;
			bestFrontier = frontier;
			bestDistance = distanceToRobot;
		}
		
		delete[] plan;
		if(bestRank == 0) break; // Frontiers are ordered by distance, so this will not improve anymore.
	}
	
	// 3. Assignment to a frontier
	ROS_DEBUG("[MinPos] Best frontier is %d.", bestFrontier);
	
	if(bestFrontier >= mFrontiers.size())
	{
		ROS_ERROR("[MinPos] Could not determine a best frontier. (Best: %d, Available: %d)", bestFrontier+1, (int)mFrontiers.size());
		return EXPL_FAILED;
	}
	
	// 4. Navigation towards the assigned frontiers for a fixed time period.
	// - This is done in Navigator.
	goal = mFrontiers.at(bestFrontier).at(0);
	return EXPL_TARGET_SET;
}

void MinPosPlanner::findCluster(GridMap* map, unsigned int startCell)
{
	// Create a new frontier and expand it
	Frontier front;
	int frontNumber = -2 - mFrontiers.size();
	int minAreaSize = mMinTargetAreaSize / (map->getResolution() * map->getResolution());
	
	// Initialize a new queue with the found frontier cell
	Queue frontQueue;
	frontQueue.insert(Entry(0.0, startCell));
	bool isBoundary = false;
	
//	Queue unexplQueue;
//	int areaSize = 0;
	
	while(!frontQueue.empty())
	{
		// Get the nearest cell from the queue
		Queue::iterator next = frontQueue.begin();
		double distance = next->first;
		unsigned int index = next->second;
		unsigned int x, y;
		frontQueue.erase(next);
		
		// Check if it is a frontier cell
		if(!map->isFrontier(index)) continue;
		
		// Add it to current frontier
		front.push_back(index);
		mFrontierCells++;
		
		// Add all adjacent cells to queue
		for(unsigned int it = 0; it < 4; it++)
		{
			int i = index + mOffset[it];
			if(map->isFree(i) && mPlan[i] == -1)
			{
				mPlan[i] = distance + map->getResolution();
				frontQueue.insert(Entry(distance + map->getResolution(), i));
			}
		}
/*
		// Calculate the size of the adjacent unknown region (as a kind of expected information gain)
		unexplQueue.insert(Entry(0.0, index));
		while(!unexplQueue.empty() && !isBoundary && areaSize <= minAreaSize)
		{
			// Get the nearest cell from the queue
			Queue::iterator next2 = unexplQueue.begin();
			double distance2 = next2->first;
			unexplQueue.erase(next2);
			
			unsigned int index2 = next2->second;
			unsigned int i2, x2, y2;
			if(!map->getCoords(x2, y2, index2))
			{
				ROS_WARN("[MinPos] Unknown cell in queue was out of map!");
				continue;
			}
			
			std::vector<unsigned int> neighbors;
			if(x2 > 0                  && map->getIndex(x2-1,y2  ,i2)) neighbors.push_back(i2); else isBoundary = true;
			if(x2 < map->getWidth()-1  && map->getIndex(x2+1,y2  ,i2)) neighbors.push_back(i2); else isBoundary = true;
			if(y2 > 0                  && map->getIndex(x2  ,y2-1,i2)) neighbors.push_back(i2); else isBoundary = true;
			if(y2 < map->getHeight()-1 && map->getIndex(x2  ,y2+1,i2)) neighbors.push_back(i2); else isBoundary = true;
			
			// Add all adjacent cells to queue
			for(unsigned int it2 = 0; it2 < neighbors.size(); it2++)
			{
				i2 = neighbors[it2];
				if(map->getData(i2) == -1 && mPlan[i2] != frontNumber)
				{
					mPlan[i2] = frontNumber;
					unexplQueue.insert(Entry(distance2 + map->getResolution(), i2));
					areaSize++;
				}
			}
		}

	}

	ROS_DEBUG("[MinPos] Size of unknown area: %d / Boundary: %d", areaSize, isBoundary);
	if(isBoundary || areaSize >= minAreaSize)
		mFrontiers.push_back(front);
*/
	}
	mFrontiers.push_back(front);
}
