cmake_minimum_required(VERSION 2.8.3)
project(activity_temporal_patterns)

find_package(catkin REQUIRED COMPONENTS
  # activity_simulation
  region_observation
  rospy
  spectral_poisson_processes
  std_msgs
  message_generation
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
  ActivityEstimateSrv.srv
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
#  LIBRARIES activity_temporal_patterns
  CATKIN_DEPENDS region_observation rospy spectral_poisson_processes std_msgs message_runtime
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
