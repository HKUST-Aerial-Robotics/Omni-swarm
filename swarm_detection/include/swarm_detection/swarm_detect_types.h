#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <map>
#include <assert.h>
#include <aruco/aruco.h>

using namespace Eigen;
using namespace ceres;

struct Camera {
    Eigen::Affine3d trans;
    double fx, fy, cx, cy;
    double k1, k2, p1, p2;
    int size_w;
    int size_h;

    template <typename T> inline
    void project_to_camera(const T point3d[3], T point[2])  const {
        point[0] = point3d[0] / point3d[2];
        point[1] = point3d[1] / point3d[2];

        T d_u[2];
        distortion(point, d_u);
        point[0] = point[0] + d_u[0];
        point[1] = point[1] + d_u[1];

        point[0] = fx * point[0] + cx;
        point[1] = fy * point[1] + cy;
    }

    template <typename T> inline
    void distortion(const T point[2], T d_u[2]) const {
        T mx2_u = point[0] * point[0];
        T my2_u = point[1] * point[1];
        T mxy_u = point[0] * point[1];
        T rho2_u = mx2_u + my2_u;
        T rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;

        d_u[0] = point[0] * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u);
        d_u[1] = point[1] * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
    }

    Vector3d pos_on_drone;
    Quaterniond att_on_drone;

    Quaterniond att() const {
        return att_on_drone;
    }

    Vector3d pos() const {
        return pos_on_drone;
    }
};

struct DroneMarker {
    //Trans is relative to own drone
    Eigen::Affine3d trans;
    int id;
    int drone_id;
    double size;

    //Corner position on the drone
    Eigen::Vector3d rel_corner_pos(int corner_no) const {
    switch (corner_no) {
        case 0:
            return Eigen::Vector3d(0, size/2, size/2);
            break;
        case 1:
            return Eigen::Vector3d(0, -size/2, size/2);
            break;
        case 2:
            return Eigen::Vector3d(0, -size/2, -size/2);
            break;
        case 3:
            return Eigen::Vector3d(0, size/2, -size/2);
            break;
    }
    return Eigen::Vector3d(0, 0, 0);
}

    DroneMarker(int _id, int _drone_id, Eigen::Affine3d _trans, double _size):
        size(_size), id(_id), trans(_trans), drone_id(_drone_id)
    {}

};

typedef std::map<int, DroneMarker*> marker_dict;

class MarkerCornerObervsed {
public:
    int id;
    int corner_no;
    Eigen::Vector2d observed_point;
    DroneMarker * marker = nullptr;
    Eigen::Vector3d rel_corner_pos() const {
        assert(marker!=nullptr && "Must like corner to a marker before use relative corner position");
        return marker->rel_corner_pos(this->id);
    }

    MarkerCornerObervsed(int _id, int _corner_no, DroneMarker * _marker):
        id(_id), corner_no(_corner_no), marker(_marker)
    {

    }
};

struct SwarmDroneDefs {
    int drone_id;
    std::map<int, DroneMarker> markers;
};

struct Pose {
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;

    void to_vector(double ret[]) {
        ret[0] = quat.w();
        ret[1] = quat.x();
        ret[2] = quat.y();
        ret[3] = quat.z();

        ret[4] = pos.x();
        ret[5] = pos.y();
        ret[6] = pos.z();
    }
};

typedef std::vector<Camera*> camera_array;
typedef std::vector<MarkerCornerObervsed> corner_array;
typedef std::vector<aruco::Marker> aruco_marker_array;
