#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

class GridMap
{
public:
	void update(nav_msgs::OccupancyGrid grid)
	{
		mOccupancyGrid = grid;
		mMapWidth = mOccupancyGrid.info.width;
		mMapHeight = mOccupancyGrid.info.height;
		ROS_DEBUG("Got new map of size %d x %d", mMapWidth, mMapHeight);
	}

	unsigned int getWidth() {return mMapWidth;}
	unsigned int getHeight() {return mMapHeight;}
	unsigned int getSize() {return mMapWidth * mMapHeight;}
	double getResolution() {return mOccupancyGrid.info.resolution;}
	double getOriginX() {return mOccupancyGrid.info.origin.position.x;}
	double getOriginY() {return mOccupancyGrid.info.origin.position.y;}
	
	signed char getLethalCost(){return mLethalCost;}
	void setLethalCost(signed char c){mLethalCost = c;}
	
	const nav_msgs::OccupancyGrid& getMap() const {return mOccupancyGrid;}
	
	// Get the array index from the given x,y coordinates
	bool getIndex(unsigned int x, unsigned int y, unsigned int &i)
	{
		if(x >= mMapWidth || y >= mMapHeight)
		{
			return false;
		}
		i = y * mMapWidth + x;
		return true;
	}
	
	// Get the x,y coordinates from the given array index
	bool getCoordinates(unsigned int &x, unsigned int &y, unsigned int i)
	{
		if(i >= mMapWidth * mMapHeight)
		{
			ROS_ERROR("getCoords() failed!");
			return false;
		}
		y = i / mMapWidth;
		x = i % mMapWidth;
		return true;
	}
	
	// Index based methods
	signed char getData(unsigned int index)
	{
		if(index < mMapWidth * mMapHeight)
			return mOccupancyGrid.data[index];
		else
			return -1;
	}
	
	bool setData(unsigned int index, signed char value)
	{
		if(index >= mMapWidth * mMapHeight)
		{
			return false;
		}
		mOccupancyGrid.data[index] = value;
		return true;
	}

	bool isFree(unsigned int index)
	{
		signed char value = getData(index);
		if(value >= 0 && value < mLethalCost) return true;
		return false;
	}

	bool isFrontier(unsigned int index)
	{
		int y = index / mMapWidth;
		int x = index % mMapWidth;
		
		if(getData(x-1, y-1) == -1) return true;
		if(getData(x-1, y  ) == -1) return true;
		if(getData(x-1, y+1) == -1) return true;
		if(getData(x  , y-1) == -1) return true;
		if(getData(x  , y+1) == -1) return true;
		if(getData(x+1, y-1) == -1) return true;
		if(getData(x+1, y  ) == -1) return true;
		if(getData(x+1, y+1) == -1) return true;
		
		return false;
	}
	
	/** Gets indices of all free neighboring cells with given offset */ 
	std::vector<unsigned int> getFreeNeighbors(unsigned int index, int offset = 1)
	{
		std::vector<unsigned int> neighbors;
		
		if(offset < 0) offset *= -1;
		int y = index / mMapWidth;
		int x = index % mMapWidth;
		
		for(int i = -offset; i <= offset; i++)
			for(int j = -offset; j <= offset; j++)
				if(getIndex(x+i, y+j, index) && isFree(index))
					neighbors.push_back(index);
					
		return neighbors;
	}

	/** Gets indices of all neighboring cells */ 
	std::vector<unsigned int> getNeighbors(unsigned int index, bool diagonal = false)
	{
		std::vector<unsigned int> neighbors;
	
		int y = index / mMapWidth;
		int x = index % mMapWidth;
		unsigned int i;
		if(getIndex(x-1,y,  i)) neighbors.push_back(i);
		if(getIndex(x+1,y,  i)) neighbors.push_back(i);
		if(getIndex(x,  y-1,i)) neighbors.push_back(i);
		if(getIndex(x,  y+1,i)) neighbors.push_back(i);
		
		if(diagonal)
		{
			if(getIndex(x-1,y-1,i)) neighbors.push_back(i);
			if(getIndex(x-1,y+1,i)) neighbors.push_back(i);
			if(getIndex(x+1,y-1,i)) neighbors.push_back(i);
			if(getIndex(x+1,y+1,i)) neighbors.push_back(i);
		}
		return neighbors;
	}

	// Coordinate based methods
	signed char getData(int x, int y)
	{
		if(x < 0 ||x >= (int)mMapWidth || y < 0 || y >= (int)mMapHeight)
			return -1;
		else
			return mOccupancyGrid.data[y*mMapWidth + x];
	}
	
	bool setData(int x, int y, signed char value)
	{
		if(x < 0 ||x >= (int)mMapWidth || y < 0 || y >= (int)mMapHeight)
		{
			return false;
		}
		mOccupancyGrid.data[y*mMapWidth + x] = value;
		return true;
	}
	
	bool isFree(int x, int y)
	{
		signed char value = getData(x, y);
		if(value >= 0 && value < mLethalCost) return true;
		return false;
	}
private:

	nav_msgs::OccupancyGrid mOccupancyGrid;
	unsigned int mMapWidth;
	unsigned int mMapHeight;
	signed char mLethalCost;
};

#endif
