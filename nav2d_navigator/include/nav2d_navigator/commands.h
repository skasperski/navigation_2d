// Definition of commands and possible results for Navigator command service
#ifndef NAVIGATOR_COMMANDS_H
#define NAVIGATOR_COMMANDS_H

#define NAV_STOP_SERVICE "Stop"
#define NAV_PAUSE_SERVICE "Pause"
#define NAV_EXPLORE_SERVICE "StartExploration"
#define NAV_GETMAP_SERVICE  "StartMapping"
#define NAV_GOAL_TOPIC      "goal"
#define NAV_STATUS_TOPIC    "nav_status"
#define NAV_MOVE_ACTION     "MoveTo"
#define NAV_EXPLORE_ACTION  "Explore"
#define NAV_GETMAP_ACTION   "GetFirstMap"
#define NAV_LOCALIZE_ACTION "Localize"

#define NAV_ST_IDLE	      0
#define NAV_ST_NAVIGATING 1
#define NAV_ST_EXPLORING  4
#define NAV_ST_WAITING    5
#define NAV_ST_RECOVERING 6

#endif
