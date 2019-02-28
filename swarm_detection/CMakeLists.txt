cmake_minimum_required(VERSION 2.8.3)
project(swarm_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR /usr/share/OpenCV)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  swarm_msgs
  cv_bridge
  camera_models
)
find_package(OpenCV 3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES swarm_detection
 CATKIN_DEPENDS roscpp rospy std_msgs swarm_msgs cv_bridge
 DEPENDS system_lib
)

###########
## Build ##
###########

# message("cv !!!!!!!!!!!!!!!!!" ${OpenCV_INCLUDE_DIRS} ${OpenCV_LIBS})
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CERES_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${camera_model_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/drone_pose_estimator.cpp include/swarm_detection/drone_pose_estimator.h
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/swarm_detection_node.cpp)
add_executable(${PROJECT_NAME}_test src/swarm_detection_test.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} aruco)

# message("VINS Lib"${camera_model_LIBRARIES} ${camera_model_INCLUDE_DIRS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${camera_models_LIBRARIES}
  aruco
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  # opencv_hi
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  aruco
  ${PROJECT_NAME}
  ${camera_models_LIBRARIES}

)
