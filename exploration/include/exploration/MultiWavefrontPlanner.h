#ifndef MULTIWAVEFRONTPLANNER_H_
#define MULTIWAVEFRONTPLANNER_H_

#include "ExplorationPlanner.h"

class MultiWavefrontPlanner : public ExplorationPlanner
{
public:
        MultiWavefrontPlanner();
        ~MultiWavefrontPlanner();
        
        int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);
        
private:
        int mRobotID;
        bool mWaitForOthers;
        
        ros::Publisher mWavefrontPublisher;
        ros::Publisher mOtherWavefrontPublisher;
        
        RobotList mRobotList;
        
        std::string mMapFrame;
}; 

#endif // MULTIWAVEFRONTPLANNER_H_
