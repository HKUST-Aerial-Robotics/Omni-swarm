cmake_minimum_required(VERSION 2.8.3)
project(swarm_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  swarmtal_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        swarm_fused.msg
        swarm_fused_relative.msg
        swarm_remote_command.msg
        swarm_drone_source_data.msg
        node_detected_xyzyaw.msg
        node_frame.msg
        swarm_frame.msg
        node_detected.msg
        swarm_detected.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  nav_msgs 
  geometry_msgs
  std_msgs
  sensor_msgs
  swarmtal_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES swarm_msgs
#  CATKIN_DEPENDS message_generation message_runtime nav_msgs odometry_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

