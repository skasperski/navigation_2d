cmake_minimum_required(VERSION 2.8.3)
project(nav2d_karto)

# Use Thread-Building-Blocks [OpenKarto]
add_definitions(-DUSE_TBB)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav2d_msgs
  roscpp
  nav2d_localizer
  tf
)

list(APPEND CMAKE_MODULE_PATH ${nav2d_karto_SOURCE_DIR}/cmake_modules)

find_package(Eigen REQUIRED)
find_package(CSparse REQUIRED)
find_package(G2O)
find_package(Cholmod)

if(G2O_FOUND)
  add_definitions(-DUSE_G2O)
endif(G2O_FOUND)

if(CHOLMOD_FOUND)
  add_definitions(-DSBA_CHOLMOD)
endif(CHOLMOD_FOUND)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES OpenKarto MultiMapper
 CATKIN_DEPENDS nav2d_msgs roscpp nav2d_localizer tf
 DEPENDS Eigen CSparse
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

add_library(OpenKarto ${KARTO_SOURCE_FILES})

## Build MultiMapper library
include_directories(
  include
  include/${PROJECT_NAME}
  OpenKarto/source
  ${Eigen_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
add_library(MultiMapper src/MultiMapper.cpp)

## Build the mapper-node executable
set(MAPPER_INCLUDE_FILES include include/${PROJECT_NAME} OpenKarto/source)
set(MAPPER_SOURCE_FILES src/MapperNode.cpp src/SpaSolver.cpp src/spa2d.cpp src/csparse.cpp)

if(G2O_FOUND)
  set(MAPPER_INCLUDE_FILES ${MAPPER_INCLUDE_FILES} ${G2O_INCLUDE_DIR})
  set(MAPPER_SOURCE_FILES ${MAPPER_SOURCE_FILES} src/G2oSolver.cpp)
endif(G2O_FOUND)

include_directories(${MAPPER_INCLUDE_FILES})
add_executable(mapper ${MAPPER_SOURCE_FILES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(MultiMapper OpenKarto SelfLocalizer)
add_dependencies(mapper MultiMapper)

## Specify libraries to link a library or executable target against
target_link_libraries(OpenKarto tbb)
target_link_libraries(MultiMapper OpenKarto ${catkin_LIBRARIES})
target_link_libraries(mapper
  MultiMapper
  ${CSPARSE_LIBRARY}
  ${catkin_LIBRARIES}
)

if(CHOLMOD_FOUND)
target_link_libraries(mapper
  ${CHOLMOD_LIBRARIES})
endif(CHOLMOD_FOUND)

if(G2O_FOUND)
target_link_libraries(mapper
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION})
endif(G2O_FOUND)


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
