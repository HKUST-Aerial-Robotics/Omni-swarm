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
#include "swarm_msgs/swarm_types.hpp"
#include <ros/ros.h>
#include "swarm_localization_params.hpp"

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace Swarm;
using namespace Eigen;

#define AUTODIFF_STRIDE 4
// Pose in this file use only x, y, z, yaw
//                            0  1  2   3

//idstamppose[id][stamp] -> pose index in poses
typedef std::map<int64_t, std::map<int, int>> IDStampPose;



template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

template<typename T>
inline void pose_error(const T *posea, const T *poseb, T *error,
                       Eigen::Vector3d pos_std = Eigen::Vector3d(0.01, 0.01, 0.01),
                       double ang_std = 0.01) {
    error[0] = (posea[0] - poseb[0]) / pos_std.x();
    error[1] = (posea[1] - poseb[1]) / pos_std.y();
    error[2] = (posea[2] - poseb[2]) / pos_std.z();
    error[3] = NormalizeAngle(poseb[3] - posea[3]) / ang_std;
}

template<typename T>
inline void pose_error_4d(const Eigen::Matrix<T, 4, 1> & posea, 
    const Eigen::Matrix<T, 4, 1> & poseb, 
    const Eigen::Matrix<T, 4, 4> &_sqrt_inf_mat, 
    T *error) {
    Eigen::Map<Eigen::Matrix<T, 4, 1>> err(error);
    err = poseb - posea;
    err(3) = NormalizeAngle(err(3));
    err.applyOnTheLeft(_sqrt_inf_mat);
}


template<typename T>
inline void position_error(const T *posea, const T *poseb, T *error,
                       Eigen::Vector3d pos_std = Eigen::Vector3d(0.01, 0.01, 0.01)) {
    error[0] = (posea[0] - poseb[0]) / pos_std.x();
    error[1] = (posea[1] - poseb[1]) / pos_std.y();
    error[2] = (posea[2] - poseb[2]) / pos_std.z();
}


template<typename T>
inline void unit_position_error(const T *posea, const T *poseb, const double * tangent_base, T *error) {
    //For this residual; we assume poseb a unit vector
    const T _inv_dep = 1.0/sqrt(posea[0]*posea[0] +  posea[1]*posea[1] +  posea[2]*posea[2]);
    const T err0 = posea[0]*_inv_dep - poseb[0];
    const T err1 = posea[1]*_inv_dep - poseb[1];
    const T err2 = posea[2]*_inv_dep - poseb[2];

    error[0] = (tangent_base[0] * err0 + tangent_base[1] * err1 + tangent_base[2] * err2) / (T)(DETECTION_SPHERE_STD);
    error[1] = (tangent_base[3] * err0 + tangent_base[4] * err1 + tangent_base[5] * err2) / (T)(DETECTION_SPHERE_STD);
    // std::cout << "invdep\t" << _inv_dep << "\tErr0\t" << err0 << "\tEr1\t" << err1 << "\tErr2\t" << err2 <<std::endl;
    // std::cout << "error[0]\t" << error[0] << "\terror[1]\t" << error[1] <<std::endl;
}

template<typename T>
inline void unit_position_error_inv_dep(const T *posea, const T *poseb, const T inv_dep, const double * tangent_base, T *error) {
    //For this residual; we assume poseb a unit vector
    const T _inv_dep = 1.0/sqrt(posea[0]*posea[0] +  posea[1]*posea[1] +  posea[2]*posea[2]);
    const T err0 = posea[0]*_inv_dep - poseb[0];
    const T err1 = posea[1]*_inv_dep - poseb[1];
    const T err2 = posea[2]*_inv_dep - poseb[2];

    error[0] = (tangent_base[0] * err0 + tangent_base[1] * err1 + tangent_base[2] * err2) / (T)(DETECTION_SPHERE_STD);
    error[1] = (tangent_base[3] * err0 + tangent_base[4] * err1 + tangent_base[5] * err2) / (T)(DETECTION_SPHERE_STD);
    error[2] = (inv_dep - _inv_dep) / (T)(DETECTION_INV_DEP_STD);

    // std::cout << "Pa" << posea[0] << " " << posea[1] << " " << posea[2] << std::endl;
    // std::cout << "Pb" << poseb[0] << " " << poseb[1] << " " << poseb[2] << std::endl;
    // std::cout << "err" << err0 << " " << err1 << " " << err2 << std::endl;
    // std::cout << "Error" << error[0] << " " << error[1] << " " << error[2] << std::endl;
}

template<typename T>
inline void unit_position_error(const T *posea, const T *poseb, const T dep, const double * tangent_base, T *error) {
    //For this residual; we assume poseb a unit vector
    const T _dep = sqrt(posea[0]*posea[0] +  posea[1]*posea[1] +  posea[2]*posea[2]);
    const T err0 = (posea[0]/_dep - poseb[0]);
    const T err1 = (posea[1]/_dep - poseb[1]);
    const T err2 = (posea[2]/_dep - poseb[2]);

    error[0] = (tangent_base[0] * err0 + tangent_base[1] * err1 + tangent_base[2] * err2) / (T)(DETECTION_SPHERE_STD);
    error[1] = (tangent_base[3] * err0 + tangent_base[4] * err1 + tangent_base[5] * err2) / (T)(DETECTION_SPHERE_STD);
    error[2] = (_dep - dep) / (T)(DETECTION_DEP_STD);

    // std::cout << "Pa" << posea[0] << " " << posea[1] << " " << posea[2] << std::endl;
    // std::cout << "Pb" << poseb[0] << " " << poseb[1] << " " << poseb[2] << std::endl;
    // std::cout << "err" << err0 << " " << err1 << " " << err2 << std::endl;
    // std::cout << "Error" << error[0] << " " << error[1] << " " << error[2] << std::endl;
}
template<typename T>
inline void YawRotatePoint(T yaw, const T * vec, T * ret) {
    ret[0] = cos(yaw) * vec[0] - sin(yaw)*vec[1];
    ret[1] = sin(yaw) * vec[0] + cos(yaw)*vec[1];
    ret[2] = vec[2];
}

template<typename T>
inline void PoseTransformPoint(const T * pose, const T * point, T * ret) {
    YawRotatePoint(pose[3], point, ret);
    ret[0] = ret[0] + pose[0];
    ret[1] = ret[1] + pose[1];
    ret[2] = ret[2] + pose[2];
}


//dpose = a^-1.b
template<typename T>
inline void DeltaPose(const T *posea, const T *poseb, T *dpose) {
    dpose[3] = NormalizeAngle(poseb[3] - posea[3]);
    T tmp[3];

    tmp[0] = poseb[0] - posea[0];
    tmp[1] = poseb[1] - posea[1];
    tmp[2] = poseb[2] - posea[2];

    YawRotatePoint(-posea[3], tmp, dpose);
}


template<typename T>
inline void DeltaPose_Naive(const T *posea, const T *poseb, T *dpose) {
    T tmp[3];
    tmp[0] = poseb[0] - posea[0];
    tmp[1] = poseb[1] - posea[1];
    tmp[2] = poseb[2] - posea[2];

    YawRotatePoint(-posea[3], tmp, dpose);
}


//dpose = a.b
template<typename T>
inline void PoseMulti(const T *posea, const T *poseb, T *pose) {
    pose[3] = NormalizeAngle(poseb[3] + posea[3]);
    T tmp[3];
    YawRotatePoint(posea[3], poseb, tmp);
    pose[0] = tmp[0] + posea[0];
    pose[1] = tmp[1] + posea[1];
    pose[2] = tmp[2] + posea[2];
}


//dpose = a.b
template<typename T>
inline void PoseMulti_2(const T *posea, const T *posb, T *pose) {
    pose[3] = posea[3];
    T tmp[3];
    YawRotatePoint(posea[3], posb, tmp);
    pose[0] = tmp[0] + posea[0];
    pose[1] = tmp[1] + posea[1];
    pose[2] = tmp[2] + posea[2];
}

template<typename T>
inline void EigenVec2T(const Eigen::Vector3d & _p, T *p) {
    p[0] = T(_p.x());
    p[1] = T(_p.y());
    p[2] = T(_p.z());
}

template<typename T>
inline void EigenTanbase2T(const Eigen::Matrix<double, 2, 3> & _tan_base, T *tan_base) {
    tan_base[0] = T(_tan_base(0, 0));
    tan_base[1] = T(_tan_base(0, 1));
    tan_base[2] = T(_tan_base(0, 2));
    tan_base[3] = T(_tan_base(1, 0));
    tan_base[4] = T(_tan_base(1, 1));
    tan_base[5] = T(_tan_base(1, 2));
}

struct DistanceMeasurementFactor {
    double distance_measurement;
    double distance_sqrt_inf;
    DistanceMeasurementFactor(double _distance_measurement, double _distance_sqrt_inf): 
        distance_measurement(_distance_measurement), distance_sqrt_inf(_distance_sqrt_inf)
    {}

public:
 template<typename T>
    bool operator()(const T* const p_a_ptr, const T* const p_b_ptr, T *_residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> T_a(p_a_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> T_b(p_b_ptr);
        _residual[0] = ((T_a - T_b).norm() - distance_measurement)*distance_sqrt_inf;
        return true;
    }

    static ceres::CostFunction* Create(double _distance_measurement, double _distance_sqrt_inf) {
        // std::cout << "Loop" << "sqrt_inf\n" << loop->get_sqrt_information_4d() << std::endl;
        return new ceres::AutoDiffCostFunction<DistanceMeasurementFactor, 1, 4, 4>(
            new DistanceMeasurementFactor(_distance_measurement, _distance_sqrt_inf));
    }
};

class RelativePoseFactor4d {
    Swarm::Pose relative_pose;
    Eigen::Vector4d relative_pose_4d;
    Eigen::Matrix4d sqrt_inf;
    RelativePoseFactor4d(const Swarm::Pose & _relative_pose, const Eigen::Matrix4d & _sqrt_inf):
        relative_pose(_relative_pose), sqrt_inf(_sqrt_inf)
    {
        relative_pose.to_vector_xyzyaw(relative_pose_4d.data());
    }

public:
    template<typename T>
    bool operator()(const T* const p_a_ptr, const T* const p_b_ptr, T *_residual) const {
        Eigen::Map<const Eigen::Matrix<T, 4, 1>> pose_a(p_a_ptr);
        Eigen::Map<const Eigen::Matrix<T, 4, 1>> pose_b(p_b_ptr);
        Eigen::Matrix<T, 4, 1> relpose_est;
        const Eigen::Matrix<T, 4, 1> _relative_pose_4d = relative_pose_4d.template cast<T>();
        const Eigen::Matrix<T, 4, 4> _sqrt_inf = sqrt_inf.template cast<T>();
        DeltaPose(pose_a.data(), pose_b.data(), relpose_est.data());
        pose_error_4d(relpose_est, _relative_pose_4d, _sqrt_inf, _residual);

        return true;
    }

    static ceres::CostFunction* Create(const Swarm::Pose & _relative_pose, const Eigen::Matrix4d & _sqrt_inf) {
        return new ceres::AutoDiffCostFunction<RelativePoseFactor4d, 4, 4, 4>(
            new RelativePoseFactor4d(_relative_pose, _sqrt_inf));
    }

    static ceres::CostFunction* CreateCov6d(const Swarm::Pose & _relative_pose, const Eigen::Matrix6d & cov) {
        Matrix4d cov4d = Matrix4d::Zero();
        cov4d.block<3, 3>(0, 0) = cov.block<3, 3>(0, 0);
        cov4d(3, 3) = cov(5, 5);
        auto _sqrt_inf_4d = cov4d.inverse().cwiseAbs().cwiseSqrt();;
        // std::cout << "Odom" << "sqrt_inf\n" << _sqrt_inf_4d << std::endl;
        return new ceres::AutoDiffCostFunction<RelativePoseFactor4d, 4, 4, 4>(
            new RelativePoseFactor4d(_relative_pose, _sqrt_inf_4d));
    }

    static ceres::CostFunction* Create(const Swarm::GeneralMeasurement2Drones* _loc) {
        auto loop = static_cast<const Swarm::LoopEdge*>(_loc);
        // std::cout << "Loop" << "sqrt_inf\n" << loop->get_sqrt_information_4d() << std::endl;
        return new ceres::AutoDiffCostFunction<RelativePoseFactor4d, 4, 4, 4>(
            new RelativePoseFactor4d(loop->relative_pose, loop->get_sqrt_information_4d()));
    }
};

class DroneDetection4dFactor {
    bool enable_depth;
    Swarm::DroneDetection det;
    bool enable_dpose;
    Eigen::Vector3d dir;
    double inv_dep;
    double dep;
    bool use_inv_dep = true;

    Eigen::Vector4d dposea, dposeb;
    bool enable_dpose_b = false;
    DroneDetection4dFactor(const Swarm::DroneDetection & _det): det(_det) {
        enable_depth = det.enable_depth;
        enable_dpose = det.enable_dpose;
        dir = det.p;
        inv_dep = det.inv_dep;
        dep = 1/inv_dep;

        if (enable_dpose) {
            det.dpose_self_a.to_vector_xyzyaw(dposea.data());
            det.dpose_self_b.to_vector_xyzyaw(dposeb.data());
        }

        // ROS_INFO("[SWARM_LOCAL](DetectionFactor) Detection %d->%d@%ld enable_depth %d inv_dep %d dir [%+3.2f, %+3.2f, %+3.2f] enable_dpose %d dposea %s dposeb %s", 
        //     _det.id_a, _det.id_b, TSShort(_det.ts_a), enable_depth, use_inv_dep,
        //     dir.x(), dir.y(), dir.z(), 
        //     enable_dpose, det.dpose_self_a.tostr().c_str(), det.dpose_self_b.tostr().c_str());
    }

public:
    template<typename T>
    bool operator()(const T* const p_a_ptr, const T* const p_b_ptr, T *_residual) const {
        Eigen::Matrix<T, 3, 1> relpose_est;

        Eigen::Map<const Eigen::Matrix<T, 4, 1>> pose_a(p_a_ptr);
        Eigen::Map<const Eigen::Matrix<T, 4, 1>> pose_b(p_b_ptr);

        if (enable_dpose) {
            Eigen::Matrix<T, 4, 1> _pose_a;
            Eigen::Matrix<T, 4, 1> _pose_b;
            const Eigen::Matrix<T, 4, 1> _dposea = dposea.template cast<T>();
            PoseMulti(pose_a.data(), _dposea.data(), _pose_a.data());
            const Eigen::Matrix<T, 4, 1> _dposeb = dposeb.template cast<T>();
            PoseMulti(pose_b.data(), _dposeb.data(), _pose_b.data());
            DeltaPose_Naive(_pose_a.data(), _pose_b.data(), relpose_est.data());
        } else {
            Eigen::Matrix<T, 4, 1> _pose_a = pose_a;
            _pose_a(2) = _pose_a(2) + T(det.extrinsic.pos().z()); //Not accurate
            DeltaPose_Naive(_pose_a.data(), pose_b.data(), relpose_est.data());
        }
        
        
        Eigen::Matrix<T, 3, 1> rel_p = dir.template cast<T>();

        const double * tan_base = det.detect_tan_base.data();

        if (enable_depth) {
            if (use_inv_dep) {
                T inv_dep = (T)(this->inv_dep);
                unit_position_error_inv_dep(relpose_est.data(), rel_p.data(), inv_dep, tan_base, _residual);
            } else {
                T dep = (T)(this->dep);
                unit_position_error(relpose_est.data(), rel_p.data(), dep, tan_base, _residual);
            }
        } else {
            // std::cout << "relpose_est normed" << relpose_est.transpose()/relpose_est.norm() << "rel_p" << rel_p.transpose() << std::endl;
            unit_position_error(relpose_est.data(), rel_p.data(), tan_base, _residual);
        }

        return true;    
    }


    static ceres::CostFunction* Create(const Swarm::GeneralMeasurement2Drones* _loc) {
        auto det = static_cast<const Swarm::DroneDetection*>(_loc);
        // std::cout << "Loop" << "sqrt_inf\n" << loop->get_sqrt_information_4d() << std::endl;
        int res_count = 2;
        if (det->enable_depth) {
            res_count = 3;
        }

        return new ceres::AutoDiffCostFunction<DroneDetection4dFactor, ceres::DYNAMIC, 4, 4>(
            new DroneDetection4dFactor(*det), res_count);
    }

    static ceres::CostFunction* Create(const Swarm::DroneDetection & _det) {
        // std::cout << "Loop" << "sqrt_inf\n" << loop->get_sqrt_information_4d() << std::endl;
        int res_count = 2;
        if (_det.enable_depth) {
            res_count = 3;
        }
        return new ceres::AutoDiffCostFunction<DroneDetection4dFactor, ceres::DYNAMIC, 4, 4>(
            new DroneDetection4dFactor(_det), res_count);
    }
};

