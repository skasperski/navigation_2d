cmake_minimum_required(VERSION 2.8.3)
project(robot_navigator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  tf
  actionlib
  actionlib_msgs
  geometry_msgs
  robot_operator
  nav2d_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Explore.action
  GetFirstMap.action
  Localize.action
  MoveToPosition2D.action
)

add_service_files(FILES SendCommand.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES RobotNavigator MapInflationTool
 CATKIN_DEPENDS robot_operator roscpp tf message_runtime actionlib_msgs geometry_msgs nav2d_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(MapInflationTool src/MapInflationTool.cpp)
add_library(RobotNavigator src/RobotNavigator.cpp)

## Declare a cpp executable
add_executable(navigator src/navigator.cpp src/RobotNavigator.cpp)
add_executable(set_goal_client src/set_goal_client.cpp)
add_executable(explore_client src/explore_client.cpp)
add_executable(get_map_client src/get_map_client.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(RobotNavigator robot_navigator_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(RobotNavigator MapInflationTool ${catkin_LIBRARIES})
target_link_libraries(navigator RobotNavigator ${catkin_LIBRARIES})
target_link_libraries(set_goal_client ${catkin_LIBRARIES})
target_link_libraries(explore_client ${catkin_LIBRARIES})
target_link_libraries(get_map_client ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS RobotNavigator MapInflationTool navigator set_goal_client explore_client get_map_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_navigator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
