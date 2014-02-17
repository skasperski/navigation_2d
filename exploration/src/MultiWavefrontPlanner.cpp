#include "exploration/MultiWavefrontPlanner.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/GridCells.h"

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

using namespace ros;

MultiWavefrontPlanner::MultiWavefrontPlanner()
{
	NodeHandle robotNode;
	robotNode.param("robot_id", mRobotID, 1);
	robotNode.param("map_frame", mMapFrame, std::string("map"));
	
	tf::TransformListener tf;
	std::string tfPrefix = tf.getTFPrefix();
	mMapFrame = tf::resolve(tfPrefix, mMapFrame);
	
	NodeHandle navigatorNode("~/");
	mWaitForOthers = false;
	
	// Initialize Publisher/Subscriber
	mWavefrontPublisher = navigatorNode.advertise<nav_msgs::GridCells>("wave", 1);
	mOtherWavefrontPublisher = navigatorNode.advertise<nav_msgs::GridCells>("others", 1);
}

MultiWavefrontPlanner::~MultiWavefrontPlanner()
{
	
}

int MultiWavefrontPlanner::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{
	// Initialize a search map
	unsigned int mapSize = map->getSize();
	int* searchMap = new int[mapSize];
	for(unsigned int i = 0; i < mapSize; i++)
	{
		searchMap[i] = 0;
	}
	
	// Initialize the queue with the robot position
	Queue queue;
	Entry startPoint(0.0, start);
	queue.insert(startPoint);
	searchMap[start] = mRobotID;
	
	// A second queue for cells occupied by other robots
	Queue buffer;
	
	// Insert position of other robots
	PoseList l = mRobotList.getRobots();
	for(PoseList::iterator p = l.begin(); p != l.end(); p++)
	{
		if((int)p->first == mRobotID) continue;
		unsigned int robot_x =  (double)(p->second.x - map->getOriginX()) / map->getResolution();
		unsigned int robot_y =  (double)(p->second.y - map->getOriginY()) / map->getResolution();
		unsigned int robot = 0;
		if(map->getIndex(robot_x, robot_y, robot))
		{
			queue.insert(Entry(0.0, robot));
			searchMap[robot] = p->first;
		}else
		{
			ROS_WARN("Couldn't place robot %d in search map!", p->first);
		}
	}

	Queue::iterator next;
	double linear = map->getResolution();
	bool foundFrontier = false;
	bool hasFrontiers = false;
	int cellCount = 0;
	
	std::vector<std::pair<double, double> > points;
	std::vector<std::pair<double, double> > others;
	
	// Do full search with weightless Dijkstra-Algorithm
	while(!foundFrontier && !queue.empty())
	{
		cellCount++;
		// Get the nearest cell from the queue
		next = queue.begin();
		double distance = next->first;
		unsigned int index = next->second;
		queue.erase(next);
		int currentBot = searchMap[index];
		
		// Add all adjacent cells
		if(map->isFrontier(index))
		{
			hasFrontiers = true;
			if(currentBot == mRobotID)
			{
				foundFrontier = true;
				goal = index;
			}
		}else
		{
			std::vector<unsigned int> neighbors = map->getFreeNeighbors(index);
			for(unsigned int i = 0; i < neighbors.size(); i++)
			{
				unsigned int x,y;
				if(!map->getCoordinates(x,y,neighbors[i])) continue;
				if(searchMap[neighbors[i]] == 0)
				{
					queue.insert(Entry(distance+linear, neighbors[i]));
					searchMap[neighbors[i]] = currentBot;
					if(currentBot == mRobotID)
					{
						points.push_back(std::pair<double, double>(((x+0.5) * map->getResolution()) + map->getOriginX(), ((y+0.5) * map->getResolution()) + map->getOriginY()));
					}else
					{
						others.push_back(std::pair<double, double>(((x+0.5) * map->getResolution()) + map->getOriginX(), ((y+0.5) * map->getResolution()) + map->getOriginY()));
					}
				}else if(searchMap[neighbors[i]] != mRobotID && currentBot == mRobotID)
				{
					// We don't continue the dijkstra now, but keep the point for possible later use
					buffer.insert(Entry(distance+linear, neighbors[i]));
				}
			}				
		}
	}

	// If not supposed to wait, now continue the wavefront into other robots area
	if(!mWaitForOthers)
	{
		while(!foundFrontier && !buffer.empty())
		{
			cellCount++;
			// Get the nearest cell from the buffer
			next = buffer.begin();
			double distance = next->first;
			unsigned int index = next->second;
			buffer.erase(next);
			
			// Add all adjacent cells
			if(map->isFrontier(index))
			{
				foundFrontier = true;
				goal = index;
			}else
			{
				std::vector<unsigned int> neighbors = map->getFreeNeighbors(index);
				for(unsigned int i = 0; i < neighbors.size(); i++)
				{
					if(map->isFree(neighbors[i]) && searchMap[neighbors[i]] != mRobotID)
					{
						unsigned int x,y;
						if(!map->getCoordinates(x,y,neighbors[i])) continue;
						buffer.insert(Entry(distance+linear, neighbors[i]));
						searchMap[neighbors[i]] = mRobotID;
						points.push_back(std::pair<double, double>(((x+0.5) * map->getResolution()) + map->getOriginX(), ((y+0.5) * map->getResolution()) + map->getOriginY()));
					}
				}
			}
		}
	}

	delete[] searchMap;
	ROS_DEBUG("Checked %d cells.", cellCount);
	
	// GridCells for debugging in RVIZ
	nav_msgs::GridCells wave_msg;
	wave_msg.header.frame_id = mMapFrame.c_str();
	wave_msg.header.stamp = Time::now();
	wave_msg.cell_width = map->getResolution();
	wave_msg.cell_height = map->getResolution();
	wave_msg.cells.resize(points.size());
	for(unsigned int i = 0; i < points.size(); i++)
	{
		wave_msg.cells[i].x = points[i].first;
		wave_msg.cells[i].y = points[i].second;
		wave_msg.cells[i].z = 0.0;
	}
	mWavefrontPublisher.publish(wave_msg);
	
	wave_msg.cells.resize(others.size());
	for(unsigned int i = 0; i < others.size(); i++)
	{
		wave_msg.cells[i].x = others[i].first;
		wave_msg.cells[i].y = others[i].second;
		wave_msg.cells[i].z = 0.0;
	}
	mOtherWavefrontPublisher.publish(wave_msg);
	
	if(foundFrontier)
	{
		return EXPL_TARGET_SET;
	}else if(hasFrontiers)
	{
		if(buffer.size() > 0)
			return EXPL_WAITING;
		else
		{
			ROS_WARN("[MultiWave] Frontiers present, but not reachable.");
			return EXPL_FAILED;
		}
	}else
	{
		if(cellCount > 50)
			return EXPL_FINISHED;
		else
		{
			ROS_WARN("[MultiWave] No Frontiers present, but robot seems locked.");
			return EXPL_FAILED;
		}
	}
}
