cmake_minimum_required(VERSION 2.8.3)
project(nav2d_localizer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES SelfLocalizer
 CATKIN_DEPENDS roscpp tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/${PROJECT_NAME}/
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
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

add_library(SelfLocalizer 
  src/SelfLocalizer.cpp 
  ${PF_SOURCE_FILES} 
  ${MAP_SOURCE_FILES}
)

## Declare a cpp executable
add_executable(localizer src/LocalizerNode.cpp)
target_link_libraries(localizer SelfLocalizer ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS SelfLocalizer localizer
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_self_localizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
