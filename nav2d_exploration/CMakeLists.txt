cmake_minimum_required(VERSION 2.8.3)
project(nav2d_exploration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pluginlib
  geometry_msgs
  nav_msgs
  visualization_msgs
  nav2d_navigator
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES ExplorationPlugins
  CATKIN_DEPENDS
    roscpp
	tf
	pluginlib
	geometry_msgs
	nav_msgs
	visualization_msgs
	nav2d_navigator
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}/
  ${catkin_INCLUDE_DIRS}
)
add_library(ExplorationPlugins
  src/NearestFrontierPlanner.cpp
  src/MultiWavefrontPlanner.cpp
  src/MinPosPlanner.cpp
  src/exploration_plugins.cpp)

add_dependencies(ExplorationPlugins ${catkin_EXPORTED_TARGETS})

target_link_libraries(ExplorationPlugins ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS ExplorationPlugins
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  exploration.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nav2d_exploration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
