cmake_minimum_required(VERSION 2.8.3)
project(swarm_localization)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wno-write-strings -Wno-format-overflow")


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
        camera_models
        )
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(lcm REQUIRED)

# list(APPEND CMAKE_MODULE_PATH $ENV{HOME}/source/backward-cpp)

# find_package(Backward)

catkin_package(
 INCLUDE_DIRS include
        #  LIBRARIES swarm_localization
        CATKIN_DEPENDS roscpp rospy std_msgs swarm_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  /usr/local/include
)

add_executable(${PROJECT_NAME}_node
        include/swarm_localization/swarm_localization_factors.hpp
        include/swarm_localization/swarm_localization_solver.hpp
        src/swarm_localization_node.cpp
        src/localization_DA_init.cpp
        src/swarm_localization_solver.cpp
        src/swarm_outlier_rejection/swarm_outlier_rejection.cpp
        src/swarm_outlier_rejection/third_party/fast_max-clique_finder/src/findClique.cpp
        src/swarm_outlier_rejection/third_party/fast_max-clique_finder/src/findCliqueHeu.cpp
        src/swarm_outlier_rejection/third_party/fast_max-clique_finder/src/graphIO.cpp
        src/swarm_outlier_rejection/third_party/fast_max-clique_finder/src/utils.cpp
)

add_executable(${PROJECT_NAME}_simulator
        test/swarm_local_sim.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
add_dependencies(${PROJECT_NAME}_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${camera_models_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        cgraph
        dw
        lcm
)

target_link_libraries(${PROJECT_NAME}_simulator
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${camera_models_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        dw
        faiss
        OpenMP::OpenMP_CXX
)

