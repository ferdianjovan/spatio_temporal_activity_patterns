cmake_minimum_required(VERSION 2.8.3)
project(temporal_patterns_launch)

find_package(catkin REQUIRED COMPONENTS
  activity_temporal_patterns
  people_temporal_patterns
  region_observation
  rospy
  scene_temporal_patterns
  human_trajectory
)

# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES temporal_patterns_launch
  CATKIN_DEPENDS activity_temporal_patterns people_temporal_patterns region_observation rospy scene_temporal_patterns human_trajectory
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
