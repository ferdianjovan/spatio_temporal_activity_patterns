cmake_minimum_required(VERSION 2.8.3)
project(activity_recommender_system)

find_package(catkin REQUIRED COMPONENTS
  region_observation
  roslib
  rospy
  std_msgs
  strands_exploration_msgs
  strands_navigation_msgs
  people_temporal_patterns
  activity_temporal_patterns
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
  FILES
  ChangeMethodSrv.srv
)

## Generate actions in the 'action' folder
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
#  LIBRARIES activity_recommender_system
  CATKIN_DEPENDS region_observation roslib rospy strands_exploration_msgs strands_navigation_msgs people_temporal_patterns activity_temporal_patterns std_msgs
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
