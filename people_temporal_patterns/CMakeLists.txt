cmake_minimum_required(VERSION 2.8.3)
project(people_temporal_patterns)

find_package(catkin REQUIRED COMPONENTS
  human_trajectory
  region_observation
  roslib
  rospy
  std_msgs
  message_generation
  spectral_poisson_processes
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# add_message_files(
#   FILES
#   RegionObservationTime.msg
#   PoissonMsg.msg
# )

add_service_files(
  FILES
  PeopleEstimateSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES people_temporal_patterns
  CATKIN_DEPENDS human_trajectory region_observation roslib rospy spectral_poisson_processes std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# install(PROGRAMS
#   scripts/counting_manager.py
#   src/region_observation/observe.py
#   src/periodic_poisson_processes/people_poisson.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY launch
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
