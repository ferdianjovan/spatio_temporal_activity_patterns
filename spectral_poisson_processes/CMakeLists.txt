cmake_minimum_required(VERSION 2.8.3)
project(spectral_poisson_processes)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  mongodb_store
  message_generation
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  ProcessMsg.msg
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
#  LIBRARIES spectral_poisson_processes
  CATKIN_DEPENDS mongodb_store rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
