cmake_minimum_required(VERSION 2.8.3)
project(detection_observation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mongodb_store
  region_observation
  message_generation
  vision_people_logging
  simple_change_detector
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  DetectionObservation.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS mongodb_store region_observation rospy simple_change_detector message_runtime vision_people_logging
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
  scripts/online_observation.py
  src/ubd_count.py
  src/scene_count.py
  src/trajectory_count.py
  src/simulate_count.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
