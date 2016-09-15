cmake_minimum_required(VERSION 2.8.3)
project(region_observation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mongodb_store
  rospy
  std_msgs
  sensor_msgs
  soma_manager
  soma_map_manager
  message_generation
  visualization_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Observation.msg
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
#  LIBRARIES region_observation
  CATKIN_DEPENDS geometry_msgs mongodb_store rospy sensor_msgs soma_manager soma_map_manager visualization_msgs std_msgs message_runtime
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

install(PROGRAMS
  scripts/observe.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
