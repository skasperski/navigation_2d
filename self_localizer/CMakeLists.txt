cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(PF_SOURCE_FILES
  src/pf/pf.c 
  src/pf/pf_kdtree.c 
  src/pf/pf_pdf.c 
  src/pf/pf_vector.c
  src/pf/eig3.c
  src/pf/pf_draw.c
 )

SET(MAP_SOURCE_FILES
  src/map/map.c
  src/map/map_range.c
  src/map/map_store.c
  src/map/map_draw.c
  src/map/map_cspace.cpp
)

rosbuild_add_library(pf ${PF_SOURCE_FILES})
rosbuild_add_library(map ${MAP_SOURCE_FILES})

#common commands for building c++ executables and libraries
rosbuild_add_library(SelfLocalizer src/SelfLocalizer.cpp)
target_link_libraries(SelfLocalizer pf map)

rosbuild_add_executable(Localizer src/LocalizerNode.cpp)
target_link_libraries(Localizer SelfLocalizer)
