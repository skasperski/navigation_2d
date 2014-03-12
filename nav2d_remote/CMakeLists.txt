cmake_minimum_required(VERSION 2.8.3)
project(nav2d_remote)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav2d_navigator
  nav2d_operator
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    nav2d_navigator
    nav2d_operator
    roscpp
    sensor_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(remote_joy src/remote_joy.cpp)
add_executable(sim_joy src/sim_joy.cpp)

target_link_libraries(remote_joy ${catkin_LIBRARIES})
target_link_libraries(sim_joy ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS remote_joy sim_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_remote_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
