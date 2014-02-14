#include <pluginlib/class_list_macros.h>
#include <robot_navigator/ExplorationPlanner.h>
#include <exploration/NearestFrontierPlanner.h>

PLUGINLIB_DECLARE_CLASS(exploration, nearest_frontier, NearestFrontierPlanner, ExplorationPlanner)
