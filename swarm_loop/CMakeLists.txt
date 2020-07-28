cmake_minimum_required(VERSION 2.8.3)
project(swarm_loop)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  rospy
  std_msgs
  cv_bridge
  swarm_msgs
  message_generation
  camera_models
  vins
)


SET("OpenCV_DIR"  "/usr/local/share/OpenCV/")
find_package(OpenCV 3.4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lcm REQUIRED)
find_package(Backward)


add_service_files(
  FILES
  HFNetSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES swarm_loop
  CATKIN_DEPENDS std_msgs cv_bridge roscpp rosmsg rospy swarm_msgs
  #DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  /usr/local/include/
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  # ${DBoW3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet
  src/swarm_loop_node.cpp
  src/loop_cam.cpp
  src/loop_detector.cpp
  src/loop_net.cpp
  include/loop_cam.h
  include/loop_detector.h
  include/loop_net.h
  src/loop_params.cpp
)

add_executable(${PROJECT_NAME}_spy
  src/swarm_loop_spy.cpp
  src/loop_net.cpp
  src/loop_params.cpp
)




add_dependencies(${PROJECT_NAME}_nodelet
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME}_spy
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   lcm
   faiss
   dw
)


target_link_libraries(${PROJECT_NAME}_spy
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   lcm
   dw
)
