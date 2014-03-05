#include "NearestFrontierPlanner.h"

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

NearestFrontierPlanner::NearestFrontierPlanner()
{
	
}

NearestFrontierPlanner::~NearestFrontierPlanner()
{
	
}

int NearestFrontierPlanner::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{
	// Create some workspace for the wavefront algorithm
	unsigned int mapSize = map->getSize();
	double* plan = new double[mapSize];
	for(unsigned int i = 0; i < mapSize; i++)
	{
		plan[i] = -1;
	}
	
	// Initialize the queue with the robot position
	Queue queue;
	Entry startPoint(0.0, start);
	queue.insert(startPoint);
	plan[start] = 0;
	
	Queue::iterator next;
	double distance;
	double linear = map->getResolution();
	bool foundFrontier = false;
	int cellCount = 0;
	
	// Do full search with weightless Dijkstra-Algorithm
	while(!queue.empty())
	{
		cellCount++;
		// Get the nearest cell from the queue
		next = queue.begin();
		distance = next->first;
		unsigned int index = next->second;
		queue.erase(next);
		
		// Add all adjacent cells
		if(map->isFrontier(index))
		{
			// We reached the border of the map, which is unexplored terrain as well:
			foundFrontier = true;
			goal = index;
			break;
		}else
		{
			unsigned int ind[4];

			ind[0] = index - 1;               // left
			ind[1] = index + 1;               // right
			ind[2] = index - map->getWidth(); // up
			ind[3] = index + map->getWidth(); // down
			
			for(unsigned int it = 0; it < 4; it++)
			{
				unsigned int i = ind[it];
				if(map->isFree(i) && plan[i] == -1)
				{
					queue.insert(Entry(distance+linear, i));
					plan[i] = distance+linear;
				}
			}
		}
	}

	ROS_DEBUG("Checked %d cells.", cellCount);	
	delete[] plan;
	
	if(foundFrontier)
	{
		return EXPL_TARGET_SET;
	}else
	{
		if(cellCount > 50)
			return EXPL_FINISHED;
		else
			return EXPL_FAILED;
	}
}
