cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)
set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} "-DSBA_CHOLMOD")

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#main mapper class
rosbuild_add_library(${PROJECT_NAME} src/MultiMapper.cpp src/SpaSolver.cpp src/spa2d.cpp src/csparse.cpp)
target_link_libraries(${PROJECT_NAME} OpenKarto cxsparse)

#multi-robot graph mapper node
rosbuild_add_executable(MultiMapper src/MapperNode.cpp)
target_link_libraries(MultiMapper ${PROJECT_NAME} tbb cholmod)
