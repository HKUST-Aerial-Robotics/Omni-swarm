#pragma once
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <map>
#include <assert.h>
#include "ceres/rotation.h"
#include "ceres/ceres.h"
#include <swarm_detection/drone_pose_costfunc.h>

class DronePoseEstimator {
    camera_array cam_defs;
    marker_dict md;
public:
    DronePoseEstimator(camera_array _ca);

    //point_by_cam is point corre to camera. length must equal to camera
    //marker is the id of these markers
    //All this marker must belong to one drone
    Eigen::Affine3d estimation_drone_pose(std::vector<corner_array> point_by_cam);
};