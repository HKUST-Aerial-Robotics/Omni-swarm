#pragma once
#include <Eigen/Eigen>
#include <assert.h>
#include <map>
#include <vector>
#include <geometry_msgs/Pose.h>
#include "Pose_t.hpp"

using namespace Eigen;

inline Eigen::Vector3d quat2eulers(const Eigen::Quaterniond &quat) {
    Eigen::Vector3d rpy;
    rpy.x() = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()),
                    1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y()));
    rpy.y() = asin(2 * (quat.w() * quat.y() - quat.z() * quat.x()));
    rpy.z() = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()),
                    1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z()));
    return rpy;
}

template <typename T>
T wrap_angle(T angle) {
    while (angle > M_PI) {
        angle = angle - 2 * M_PI;
    }

    while (angle < -M_PI) {
        angle = angle + 2 * M_PI;
    }
    return angle;
}
namespace Swarm {
class Pose {

    Eigen::Vector3d position = Eigen::Vector3d(0, 0, 0);
    Eigen::Quaterniond attitude = Eigen::Quaterniond(1, 0, 0, 0);
    Eigen::Quaterniond attitude_yaw_only = Eigen::Quaterniond(1, 0, 0, 0);
    double _yaw = 0;

    void update_yaw() {
        _yaw = wrap_angle(this->rpy().z());
        attitude_yaw_only = (Eigen::Quaterniond)AngleAxisd(_yaw, Vector3d::UnitZ());
    }

public:
    static Pose Identity() {
        return Pose();
    }

    void to_vector(double ret[]) const {
        ret[3] = attitude.w();
        ret[4] = attitude.x();
        ret[5] = attitude.y();
        ret[6] = attitude.z();

        ret[0] = position.x();
        ret[1] = position.y();
        ret[2] = position.z();
    }

    template <typename T>
    void to_vector(T ret[]) const {
        ret[3] = T(attitude.w());
        ret[4] = T(attitude.x());
        ret[5] = T(attitude.y());
        ret[6] = T(attitude.z());

        ret[0] = T(position.x());
        ret[1] = T(position.y());
        ret[2] = T(position.z());
    }

    template <typename T>
    void to_vector_xyzyaw(T ret[]) const {
        ret[0] = T(position.x());
        ret[1] = T(position.y());
        ret[2] = T(position.z());

        ret[3] = T(yaw());
    }

    template <typename T>
    void to_vector_xy(T ret[]) const {
        ret[0] = T(position.x());
        ret[1] = T(position.y());
    }


    Eigen::Vector3d apply_pose_to(Eigen::Vector3d point) const {
        return attitude * point + position;
    }

    Eigen::Vector3d rpy() const {
        return quat2eulers(attitude);
    }

    Pose(Eigen::Isometry3d trans) {
        position = trans.translation();
        attitude = trans.rotation();

        update_yaw();
    }

    Eigen::Isometry3d to_isometry() const {
        Eigen::Isometry3d a = Eigen::Translation3d(position) * attitude;
        return a;
    }

    Pose(geometry_msgs::Point pos, double yaw) {
        this->attitude = AngleAxisd(yaw, Vector3d::UnitZ());
        position.x() = pos.x;
        position.y() = pos.y;
        position.z() = pos.z;
        attitude.normalize();

        update_yaw();
    }


    Pose(Eigen::Vector3d pos, double yaw) {
        this->attitude = AngleAxisd(yaw, Vector3d::UnitZ());
        position = pos;
        attitude.normalize();

        update_yaw();
    }

    Pose(const geometry_msgs::Pose &p) {
        attitude.w() = p.orientation.w;
        attitude.x() = p.orientation.x;
        attitude.y() = p.orientation.y;
        attitude.z() = p.orientation.z;

        position.x() = p.position.x;
        position.y() = p.position.y;
        position.z() = p.position.z;
        update_yaw();
    }

    Pose(const Eigen::Matrix3d & R, const Eigen::Vector3d & T) {
        attitude = R;
        attitude.normalize();
        position = T;
        update_yaw();
    }


    Pose(const Eigen::Quaterniond & Q, const Eigen::Vector3d &T) {
        attitude = Q;
        attitude.normalize();
        position = T;
        update_yaw();
    }


    Pose(double v[], bool xyzyaw = false) {
        if (xyzyaw) {
            this->attitude = AngleAxisd(v[3], Vector3d::UnitZ());
        } else {
            attitude.w() = v[3];
            attitude.x() = v[4];
            attitude.y() = v[5];
            attitude.z() = v[6];
        }
        position.x() = v[0];
        position.y() = v[1];
        position.z() = v[2];
        attitude.normalize();

        update_yaw();
    }

    Pose(const Pose_t & pose_t) {
        attitude.x() = pose_t.orientation[0];
        attitude.y() = pose_t.orientation[1];
        attitude.z() = pose_t.orientation[2];
        attitude.w() = pose_t.orientation[3];

        position.x() = pose_t.position[0];
        position.y() = pose_t.position[1];
        position.z() = pose_t.position[2];
        update_yaw();
    }

    geometry_msgs::Pose to_ros_pose() const {
        geometry_msgs::Pose pose;
        pose.orientation.w = attitude.w();
        pose.orientation.x = attitude.x();
        pose.orientation.y = attitude.y();
        pose.orientation.z = attitude.z();
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        return pose;
    }

    Swarm::Pose inverse() const {
        return Swarm::Pose(this->to_isometry().inverse());
    }

    friend Pose operator*(Pose a, Pose b) {
        Pose p;
        // p.position = a.attitude*(b.position+ a.position);
        p.position = a.attitude * b.position + a.position;
        p.attitude = a.attitude * b.attitude;
        p.update_yaw();

        //        printf("Res manual");
        //        p.print();
        //
        //        printf("Res eigen");
        //        p.print();
        return p;
    }

    friend Eigen::Vector3d operator*(Pose a, Eigen::Vector3d point) {
        return a.attitude * point + a.position;
    }

    //A^-1B
    static Pose DeltaPose(const Pose &a, const Pose &b, bool use_yaw_only = false) {
        //Check this!!!
        Pose p;
        if (!use_yaw_only) {
            p.position = a.attitude.inverse() * (b.position - a.position);
            p.attitude = a.attitude.inverse() * b.attitude;
            p.update_yaw();
        } else {
            /*
            dpose[3] = wrap_angle(poseb[3] - posea[3]);
            T tmp[3];
            tmp[0] = poseb[0] - posea[0];
            tmp[1] = poseb[1] - posea[1];
            tmp[2] = poseb[2] - posea[2];

            YawRotatePoint(-posea[3], tmp, dpose);*/
            double dyaw = wrap_angle(b.yaw() - a.yaw());
            Eigen::Vector3d dp = b.position - a.position;
            p.attitude = (Eigen::Quaterniond)AngleAxisd(dyaw, Vector3d::UnitZ());

            p._yaw = dyaw;
            p.attitude_yaw_only = p.attitude;

            p.position.x() = cos(-a.yaw()) * dp.x() - sin(-a.yaw()) * dp.y();
            p.position.y() = sin(-a.yaw()) * dp.x() + cos(-a.yaw()) * dp.y();
            p.position.z() = dp.z();

            // p.position = AngleAxisd(-a.yaw(), Vector3d::UnitZ()) * dp;
        }

        return p;
    }

    inline double yaw() const {
        return _yaw;
    }

    void print() const {
        auto _rpy = rpy();
        printf("T %3.3f %3.3f %3.3f RPY %3.1f %3.1f %3.1f\n",
               position.x(), position.y(), position.z(),
               _rpy.x() * 57.3,
               _rpy.y() * 57.3,
               _rpy.z() * 57.3);
    }

    inline Eigen::Vector3d pos() const {
        return position;
    }

    inline Eigen::Vector3d & pos() {
        return position;
    }

    inline Eigen::Quaterniond att_yaw_only() const {
        return attitude_yaw_only;
    }

    inline Eigen::Quaterniond att() const {
        return attitude;
    }

    inline void set_att(Eigen::Quaterniond att) {
        attitude = att;
        update_yaw();
    }

    inline void set_pos(Eigen::Vector3d pos) {
        position = pos;
    }

    inline void set_yaw_only() {
        update_yaw();
        attitude = attitude_yaw_only;
    }

    Pose() {}
};

} // namespace Swarm