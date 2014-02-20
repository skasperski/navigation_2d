#ifndef MAP_INFLATION_TOOL_H
#define MAP_INFLATION_TOOL_H

#include <queue>

#include "ros/ros.h"
#include "GridMap.h"

class CellData
{
public:
	CellData(double d, double i, unsigned int sx, unsigned int sy);
	double distance;
	unsigned int index;
	unsigned int sx, sy;
};

inline bool operator<(const CellData &a, const CellData &b)
{
	return a.distance > b.distance;
}


class MapInflationTool
{
public:
	MapInflationTool();
	~MapInflationTool();
	
	void computeCaches(unsigned int radius);
	void inflateMap(GridMap* map);
	
private:
	void enqueueObstacle(unsigned int index, unsigned int sx, unsigned int sy);
	inline double distanceLookup(int mx, int my, int src_x, int src_y);
	inline char costLookup(int mx, int my, int src_x, int src_y);
	
	GridMap* mGridMap;
	
	unsigned int mCellInflationRadius;
	char** mCachedCosts;
	double ** mCachedDistances;
	
	std::priority_queue<CellData> mInflationQueue;
	unsigned char* mInflationMarkers;
	
	char mCostObstacle;
//	char mCostLethal;
};

#endif
