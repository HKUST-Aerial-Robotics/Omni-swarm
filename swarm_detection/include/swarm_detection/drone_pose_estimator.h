#pragma once
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <map>

struct Camera {
    Eigen::Affine3d trans;
    Eigen::Matrix3d cameraMatrix, distorsionCoeff;
    int size_w;
    int size_h;
};

struct DroneMarker {
    //Trans is relative to own drone
    Eigen::Affine3d trans;
    int id;
    int drone_id;
    double size;

    //Corner position on the drone
    Eigen::Vector3d rel_corner_pos(int corner_no);

    DroneMarker(int _id, int _drone_id, Eigen::Affine3d _trans, double _size):
        size(_size), id(_id), trans(_trans), drone_id(_drone_id)
    {
    }

};

struct MarkerCornerObervsed {
    int id;
    int corner_no;
    Eigen::Vector2d point;
    DroneMarker * marker;
};

typedef std::vector<Camera> camera_array;
typedef std::vector<MarkerCornerObervsed> corner_array;
typedef std::map<DroneMarker*> marker_dict;

class DronePoseEstimator {
    camera_array cam_defs;
    marker_dict md;
public:
    DronePoseEstimator(camera_array _ca);

    //point_by_cam is point corre to camera. length must equal to camera
    //marker is the id of these markers
    Eigen::Affine3d estimation_drone_pose(std::vector<corner_array> point_by_cam);
};