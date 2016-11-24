cmake_minimum_required(VERSION 2.8.3)
project(scene_temporal_patterns)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  mongodb_store
  region_observation
  simple_change_detector
  spectral_poisson_processes
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
  FILES
  SceneEstimateSrv.srv
  SceneBestTimeEstimateSrv.srv
)

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scene_temporal_patterns
 CATKIN_DEPENDS region_observation rospy simple_change_detector spectral_poisson_processes std_msgs std_srvs mongodb_store
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
