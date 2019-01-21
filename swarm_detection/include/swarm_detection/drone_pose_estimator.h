#pragma once

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <map>
#include <assert.h>
#include "ceres/rotation.h"
#include "ceres/ceres.h"
#include <swarm_detection/drone_pose_costfunc.h>
// #include
#include <aruco/aruco.h>
using namespace Eigen;

class DronePoseEstimator {
    SwarmDroneDefs remote_drone_defs;
    camera_array & self_ca;

public:
    DronePoseEstimator(SwarmDroneDefs & _r_drone_defs, camera_array & _sca);

    double estimate_loss = 10000;
    double x[7];
    Pose estimated;
    //point_by_cam is point corre to camera. length must equal to camera
    //marker is the id of these markers
    //All this marker must belong to one drone
    cv::Mat mat_to_draw;
    bool enable_drawing = true;
    double estimate_drone_pose(std::vector<corner_array> & point_by_cam);
    double estimate_drone_pose(std::vector<corner_array> & point_by_cam, Pose initial_pose);
    void draw(double x[], std::vector<corner_array> & point_by_cam);
};



// class SwarmPosesEstimator {
//     camera_array cameras_on_main_drone;
// public:
//     //Should load some configure from file
//     SwarmPosesEstimator(std::string swarm_maker_config);

//     void on_detected(aruco_marker_array ma) {};

//     void estimate_drone_pose(int drone_id, std::vector<corner_array> point_by_cam) {};

//     virtual void on_drone_pose_estimated(int drone_id, Vector3d pos, Quaterniond att) {}
//     ;
// };