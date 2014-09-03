cmake_minimum_required(VERSION 2.8.3)
project(nav2d_navigator)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  tf
  pluginlib
  actionlib
  actionlib_msgs
  geometry_msgs
  nav2d_operator
  nav2d_msgs
)

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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES RobotNavigator MapInflationTool
  CATKIN_DEPENDS
    roscpp
    message_runtime
    tf
    actionlib
    actionlib_msgs
	 geometry_msgs
	 nav2d_msgs
    nav2d_operator
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
add_dependencies(RobotNavigator nav2d_navigator_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(navigator nav2d_navigator_generate_messages_cpp)
add_dependencies(set_goal_client nav2d_navigator_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(explore_client nav2d_navigator_generate_messages_cpp)
add_dependencies(get_map_client nav2d_navigator_generate_messages_cpp)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nav2d_navigator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
