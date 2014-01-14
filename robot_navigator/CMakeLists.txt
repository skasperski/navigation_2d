cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

# Build messages for actionlib
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_library(MapInflationTool src/MapInflationTool.cpp)
rosbuild_add_executable(navigator src/navigator.cpp src/RobotNavigator.cpp src/MinPosPlanner.cpp src/NearestFrontierPlanner.cpp src/MultiWavefrontPlanner.cpp)
rosbuild_add_executable(set_goal_client src/set_goal_client.cpp)
rosbuild_add_executable(explore_client src/explore_client.cpp)
rosbuild_add_executable(get_map_client src/get_map_client.cpp)
target_link_libraries(navigator MapInflationTool)
