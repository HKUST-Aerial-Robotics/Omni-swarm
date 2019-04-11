//
// Created by xuhao on 2/27/19.
//

#ifndef PROJECT_SWARM_RELPOSE_COSTFUNC_H
#define PROJECT_SWARM_RELPOSE_COSTFUNC_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include <time.h>
#include <thread>
#include <unistd.h>
#include "swarm_vo_fuse/swarm_types.hpp"

using namespace swarm;

//Directly solve relative pose of nodes in the swarm
//More accurate when odometry drift and easy to add armarker
//We will use Auto Diff first
//The problem become a pose graph problem

struct DroneFrameDisError {
    SwarmFrame sf;

    std::vector<int> &all_nodes;
    std::map<int, int> &id_index_map;

    DroneFrameDisError(const SwarmFrame &_sf, std::vector<int> &_all_nodes, std::map<int, int> &_id_index_map) :
            sf(_sf), all_nodes(_all_nodes), id_index_map(_id_index_map) {

    }

    template<typename T>
    bool operator()(T const *const *parameters,
                    T *residuals) const {

    }

};

#endif //PROJECT_SWARM_RELPOSE_COSTFUNC_H
