cmake_minimum_required(VERSION 2.8.3)
project(slam_karto)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav2d_msgs
  roscpp
  self_localizer
  tf
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
#  INCLUDE_DIRS include
#  LIBRARIES slam_karto
#  CATKIN_DEPENDS nav2d_msgs roscpp self_localizer tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

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

include_directories(OpenKarto/source)
add_library(OpenKarto ${KARTO_SOURCE_FILES})

## Declare a cpp library
# add_library(slam_karto
#   src/${PROJECT_NAME}/slam_karto.cpp
# )

## Declare a cpp executable
# add_executable(slam_karto_node src/slam_karto_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(slam_karto_node slam_karto_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(slam_karto_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS OpenKarto
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
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
