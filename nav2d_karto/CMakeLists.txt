cmake_minimum_required(VERSION 2.8.3)
project(nav2d_karto)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav2d_msgs
  roscpp
  nav2d_localizer
  tf
)

find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES OpenKarto MultiMapper
 CATKIN_DEPENDS nav2d_msgs roscpp nav2d_localizer tf
 DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(OpenKarto/source)

## Build OpenKarto library
SET(KARTO_SOURCE_FILES
  OpenKarto/source/Any.cpp
  OpenKarto/source/Event.cpp
  OpenKarto/source/Exception.cpp
  OpenKarto/source/Geometry.cpp
  OpenKarto/source/Grid.cpp
  OpenKarto/source/GridIndexLookup.cpp
  OpenKarto/source/Identifier.cpp
  OpenKarto/source/Logger.cpp
  OpenKarto/source/Meta.cpp
  OpenKarto/source/MetaAttribute.cpp
  OpenKarto/source/MetaClass.cpp
  OpenKarto/source/MetaClassManager.cpp
  OpenKarto/source/MetaEnum.cpp
  OpenKarto/source/MetaEnumHelper.cpp
  OpenKarto/source/MetaEnumManager.cpp
  OpenKarto/source/Module.cpp
  OpenKarto/source/Mutex.cpp
  OpenKarto/source/Object.cpp
  OpenKarto/source/Objects.cpp
  OpenKarto/source/OccupancyGrid.cpp
  OpenKarto/source/OpenKarto.cpp
  OpenKarto/source/OpenMapper.cpp
  OpenKarto/source/Parameter.cpp
  OpenKarto/source/PoseTransform.cpp
  OpenKarto/source/RangeTransform.cpp
  OpenKarto/source/Referenced.cpp
  OpenKarto/source/RigidBodyTransform.cpp
  OpenKarto/source/Sensor.cpp
  OpenKarto/source/SensorData.cpp
  OpenKarto/source/SensorRegistry.cpp
  OpenKarto/source/String.cpp
  OpenKarto/source/StringHelper.cpp
)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-DSBA_CHOLMOD")
add_library(OpenKarto ${KARTO_SOURCE_FILES})

## Build MultiMapper library
include_directories(include include/${PROJECT_NAME} OpenKarto/source ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(MultiMapper src/MultiMapper.cpp src/SpaSolver.cpp src/spa2d.cpp src/csparse.cpp)

## Build the mapper-node
add_executable(mapper src/MapperNode.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(MultiMapper OpenKarto SelfLocalizer)
add_dependencies(mapper MultiMapper)

## Specify libraries to link a library or executable target against
target_link_libraries(MultiMapper OpenKarto cxsparse ${catkin_LIBRARIES})
target_link_libraries(mapper MultiMapper tbb cholmod ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS OpenKarto MultiMapper mapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY OpenKarto/source/OpenKarto
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_slam_karto.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
