#ifndef NEARESTFRONTIERPLANNER_H_
#define NEARESTFRONTIERPLANNER_H_

#include "ExplorationPlanner.h"

class NearestFrontierPlanner : public ExplorationPlanner
{
	public:
		NearestFrontierPlanner();
		~NearestFrontierPlanner();

		int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);

	private:

};

#endif // NEARESTFRONTIERPLANNER_H_
