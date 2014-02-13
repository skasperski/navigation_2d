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

SET(KARTO_SOURCE_FILES
  src/OpenKarto/Any.cpp
  src/OpenKarto/Event.cpp
  src/OpenKarto/Exception.cpp
  src/OpenKarto/Geometry.cpp
  src/OpenKarto/Grid.cpp
  src/OpenKarto/GridIndexLookup.cpp
  src/OpenKarto/Identifier.cpp
  src/OpenKarto/Logger.cpp
  src/OpenKarto/Meta.cpp
  src/OpenKarto/MetaAttribute.cpp
  src/OpenKarto/MetaClass.cpp
  src/OpenKarto/MetaClassManager.cpp
  src/OpenKarto/MetaEnum.cpp
  src/OpenKarto/MetaEnumHelper.cpp
  src/OpenKarto/MetaEnumManager.cpp
  src/OpenKarto/Module.cpp
  src/OpenKarto/Mutex.cpp
  src/OpenKarto/Object.cpp
  src/OpenKarto/Objects.cpp
  src/OpenKarto/OccupancyGrid.cpp
  src/OpenKarto/OpenKarto.cpp
  src/OpenKarto/OpenMapper.cpp
  src/OpenKarto/Parameter.cpp
  src/OpenKarto/PoseTransform.cpp
  src/OpenKarto/RangeTransform.cpp
  src/OpenKarto/Referenced.cpp
  src/OpenKarto/RigidBodyTransform.cpp
  src/OpenKarto/Sensor.cpp
  src/OpenKarto/SensorData.cpp
  src/OpenKarto/SensorRegistry.cpp
  src/OpenKarto/String.cpp
  src/OpenKarto/StringHelper.cpp
)

rosbuild_add_library(OpenKarto ${KARTO_SOURCE_FILES})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#main mapper class
rosbuild_add_library(${PROJECT_NAME} src/MultiMapper.cpp src/SpaSolver.cpp src/spa2d.cpp src/csparse.cpp)
target_link_libraries(${PROJECT_NAME} OpenKarto cxsparse)

#multi-robot graph mapper node
rosbuild_add_executable(MultiMapper src/MapperNode.cpp)
target_link_libraries(MultiMapper ${PROJECT_NAME} tbb cholmod)
