#ifndef MINPOSPLANNER_H_
#define MINPOSPLANNER_H_

#include "ExplorationPlanner.h"

class MinPosPlanner : public ExplorationPlanner
{
public:
        MinPosPlanner();
        ~MinPosPlanner();
        
        int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);
        
private:
        typedef std::vector<unsigned int> Frontier;
        typedef std::vector<Frontier> FrontierList;
        
        // Methods
        void findCluster(GridMap* map, unsigned int startCell);
        
        // ROS-Stuff
        ros::Publisher mFrontierPublisher;
        
        // Components
        RobotList mRobotList;
        FrontierList mFrontiers;
        double* mPlan;
        unsigned int mFrontierCells;

        // Parameters
        int mRobotID;
        bool mVisualizeFrontiers;
        double mMinTargetAreaSize;
        unsigned int mOffset[8];
};

#endif // MINPOSPLANNER_H_
