// Definition of commands and possible results for Navigator command service
#ifndef NAVIGATOR_COMMANDS_H
#define NAVIGATOR_COMMANDS_H

#define NAV_COMMAND_SERVICE "SendCommand"
#define NAV_EXPLORE_SERVICE "StartExploration"
#define NAV_GETMAP_SERVICE  "StartMapping"
#define NAV_GOAL_TOPIC      "goal"
#define NAV_STATUS_TOPIC    "nav_status"
#define NAV_MOVE_ACTION     "MoveTo"
#define NAV_EXPLORE_ACTION  "Explore"
#define NAV_GETMAP_ACTION   "GetFirstMap"
#define NAV_LOCALIZE_ACTION "Localize"

#define NAV_COM_STOP      0
#define NAV_COM_PAUSE     1
#define NAV_COM_EXPLORE   2
#define NAV_COM_GETMAP    3

#define NAV_RES_OK        0
#define NAV_RES_IGNORED   1
#define NAV_RES_FAILED    2
#define NAV_RES_INVALID   3

#define NAV_ST_IDLE	      0
#define NAV_ST_NAVIGATING 1
#define NAV_ST_EXPLORING  4
#define NAV_ST_WAITING    5
#define NAV_ST_RECOVERING 6

#endif
