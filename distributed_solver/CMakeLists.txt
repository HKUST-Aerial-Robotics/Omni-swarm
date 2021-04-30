cmake_minimum_required(VERSION 3.0.2)
project(distributed_solver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -g -O3)
# add_compile_options(-std=c++14 -g -O0)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  swarm_localization
)

# find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES distributed_solver
 CATKIN_DEPENDS roscpp rospy std_msgs swarm_localization
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
  #$ENV{HOME}/source/ceres/internel/
  /home/xuhao/source/ceres-solver/internal/
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/distributed_solver.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_test 
  src/distributed_solver_test.cpp
)

set_target_properties(${PROJECT_NAME}_test PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_test PREFIX "")

add_dependencies(${PROJECT_NAME}_test 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${CERES_LIBRARIES}
  dw
  ${Boost_LIBRARIES}
  # OpenMP::OpenMP_CXX
)
