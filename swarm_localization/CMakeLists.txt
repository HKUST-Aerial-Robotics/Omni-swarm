cmake_minimum_required(VERSION 2.8.3)
project(swarm_localization)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        rosmsg
        std_msgs
        geometry_msgs
        swarm_msgs
        swarm_detection
        camera_models
        )
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
 INCLUDE_DIRS include
        #  LIBRARIES swarm_localization
        CATKIN_DEPENDS roscpp rospy std_msgs swarm_msgs swarm_detection
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        include/swarm_localization/swarm_types.hpp
        include/swarm_localization/localiztion_costfunction.hpp
        include/swarm_localization/swarm_localization.hpp
        src/swarm_localization_node.cpp include/swarm_localization/swarm_relpose_costfunc.h
        src/swarm_localization.cpp)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${camera_models_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )
