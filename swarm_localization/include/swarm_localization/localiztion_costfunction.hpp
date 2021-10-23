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


template<typename T>
inline void pose_error(const T *posea, const T *poseb, T *error,
                       Eigen::Vector3d pos_std = Eigen::Vector3d(0.01, 0.01, 0.01),
                       double ang_std = 0.01) {
    error[0] = (posea[0] - poseb[0]) / pos_std.x();
    error[1] = (posea[1] - poseb[1]) / pos_std.y();
    error[2] = (posea[2] - poseb[2]) / pos_std.z();
    error[3] = wrap_angle(poseb[3] - posea[3]) / ang_std;
}

template<typename T>
inline void pose_error_4d(const Eigen::Matrix<T, 4, 1> & posea, 
    const Eigen::Matrix<T, 4, 1> & poseb, 
    const Eigen::Matrix<T, 4, 4> &_sqrt_inf_mat, 
    T *error) {
    Eigen::Map<Eigen::Matrix<T, 4, 1>> err(error);
    err = poseb - posea;
    err(3) = wrap_angle(err(3));
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
    dpose[3] = wrap_angle(poseb[3] - posea[3]);
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
    pose[3] = wrap_angle(poseb[3] + posea[3]);
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

class RelativePoseFactor4d {
    Swarm::Pose relative_pose;
    Eigen::Vector4d relative_pose_4d;
    Eigen::Matrix4d sqrt_inf;
public:

    RelativePoseFactor4d(const Swarm::Pose & _relative_pose, const Eigen::Matrix4d & _sqrt_inf):
        relative_pose(_relative_pose), sqrt_inf(_sqrt_inf)
    {
        relative_pose.to_vector_xyzyaw(relative_pose_4d.data());
    }

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

    static ceres::CostFunction* Create(const Swarm::Pose & _relative_pose, const Eigen::Matrix6d & _sqrt_inf) {
        Matrix4d _sqrt_inf_4d = Matrix4d::Zero();
        _sqrt_inf_4d.block<3, 3>(0, 0) = _sqrt_inf.block<3, 3>(0, 0);
        _sqrt_inf_4d(3, 3) = _sqrt_inf(5, 5);
        return new ceres::AutoDiffCostFunction<RelativePoseFactor4d, 4, 4, 4>(
            new RelativePoseFactor4d(_relative_pose, _sqrt_inf_4d));
    }

    static ceres::CostFunction* Create(const Swarm::GeneralMeasurement2Drones* _loc) {
        auto loop = static_cast<const Swarm::LoopEdge*>(_loc);
        return new ceres::AutoDiffCostFunction<RelativePoseFactor4d, 4, 4, 4>(
            new RelativePoseFactor4d(loop->relative_pose, loop->get_sqrt_information_4d()));
    }
};

/*
class SwarmDetectionError {
    bool enable_depth;
    Swarm::DroneDetection det;
    bool enable_dpose;
    Eigen::Vector3d dir;
    double inv_dep;
    double dep;
    bool use_inv_dep = false;
public:
    SwarmDetectionError(const Swarm::GeneralMeasurement2Drones* _loc) {
        det = *(static_cast<const Swarm::DroneDetection*>(loc));
        enable_depth = det.enable_depth;
        enable_dpose = det.enable_dpose;
        dir = det.p;
        inv_dep = det.inv_dep;
        dep = 1/inv_dep;
        // ROS_INFO("SwarmDetectionError Enable dpose %d Enable Depth %d", enable_dpose, enable_depth);
        // std::cout << "rel_p" << det.p << std::endl;
    }

    template<typename T>
    bool operator()(T const *const *_poses, T *_residual) const {
        int res_count = detection_residual( _poses, _residual);
        return true;
    }

    virtual int residual_count() {
        if (enable_depth) {
            return 3;
        } else {
            return 2;
        }
    }

protected:
    template<typename T>
    inline int detection_residual(T const *const *_poses, T *_residual) const {
        T relpose_est[3], posea[4], poseb[4];
        get_pose_a(_poses, posea);
        get_pose_b(_poses, poseb);

        if (enable_dpose) {
            T _posea[4], _poseb[4], dposea[4], dposeb[4];
            
            det.dpose_self_a.to_vector_xyzyaw(dposea);
            det.dpose_self_b.to_vector_xyzyaw(dposeb);

            PoseMulti(posea, dposea, _posea);
            PoseMulti(poseb, dposeb, _poseb);
            DeltaPose_Naive(_posea, _poseb, relpose_est);
        } else {
            // T extrinsic[3], _posea[3];
            // extrinsic[0] = T(det.extrinsic.x());
            // extrinsic[1] = T(det.extrinsic.y());
            // extrinsic[2] = T(det.extrinsic.z());
            // PoseMulti_2(posea, extrinsic, _posea);
            posea[2] = posea[2] + T(det.extrinsic.z());
            DeltaPose_Naive(posea, poseb, relpose_est);
        }
        
        
        T rel_p[3];
        rel_p[0] = T(dir.x());
        rel_p[1] = T(dir.y());
        rel_p[2] = T(dir.z());

        const double * tan_base = det.detect_tan_base.data();

        if (enable_depth) {
            // std::cout << "rel_p " << rel_p[0]  << " " << rel_p[1] << " " << rel_p[2] << std::endl;
            if (use_inv_dep) {
                T inv_dep = (T)(this->inv_dep);
                unit_position_error_inv_dep(relpose_est, rel_p, inv_dep, tan_base, _residual);
            } else {
                T dep = (T)(this->dep);
                unit_position_error(relpose_est, rel_p, dep, tan_base, _residual);
            }
            // std::cout << "_residual " << _residual[0]  << " " << _residual[1] << " " << _residual[2] << std::endl;
            return 3;
        } else {
            unit_position_error(relpose_est, rel_p, tan_base, _residual);
            return 2;
        }
    
    }
};

*/
struct SwarmFrameError {
    SwarmFrame sf;
    std::map<int, int> id2poseindex;
    std::map<int, bool> yaw_observability;
    std::map<int, double> yaw_init;
    bool detection_no_scale = false;
    bool enable_distance = false;

    template<typename T>
    inline void get_pos(int _id, T const *const *_poses, T * t_pose) const {
        int index = id2poseindex.at(_id);
        t_pose[0] =  _poses[index][0];
        t_pose[1] =  _poses[index][1];
        t_pose[2] =  _poses[index][2];
    }

    //Need add anntena position here!
    template<typename T>
    inline T node_distance(int idi, int idj, T const *const *_poses) const {
        //If consider bias here?
        T posea[3] , poseb[3];
        get_pos(idi, _poses, posea);
        get_pos(idj, _poses, poseb);

        return sqrt((poseb[0] - posea[0]) * (poseb[0] - posea[0])
                    + (poseb[1] - posea[1]) * (poseb[1] - posea[1])
                    + (poseb[2] - posea[2]) * (poseb[2] - posea[2]));
    }

    inline bool has_id(const int _id) const {
        return sf.node_id_list.find(_id) != sf.node_id_list.end();
    }

    template<typename T>
    inline int nodeframe_distance_residual(NodeFrame &_nf, T const *const *_poses, T *_residual, int res_count) const {
        for (const auto &  it : _nf.dis_map) {
            int _idj = it.first;
            if (has_id(_idj) && _nf.distance_available(_idj)) {
                T _dis = T(it.second);
                //Less accuracy on distance
                _residual[res_count] = (node_distance(_nf.id, _idj, _poses) - _dis) / ((T)(DISTANCE_STD));
                res_count++;
            } else {
            }
        }
        return res_count;
    }

    int residual_count() {
        int res_count = 0;
        for (const auto & it : sf.id2nodeframe) {
            const NodeFrame &_nf = it.second;

            if (enable_distance && _nf.frame_available && _nf.dists_available) {
                // ROS_WARN("TS %d ID %d ENABLED %ld DISMAP %ld\n", TSShort(_nf.ts), _nf.id, _nf.dis_map.size(), _nf.enabled_distance.size());
                for (auto it : _nf.dis_map) {
                    if (has_id(it.first) && _nf.distance_available(it.first)) 
                        res_count++;
                }
            }
        }
        return res_count;
    }



    template<typename T>
    bool operator()(T const *const *_poses, T *_residual) const {
        int res_count = 0;

        for (auto it : sf.id2nodeframe) {
//            auto _id = it.first;
            NodeFrame &_nf = it.second;

            //First we come to distance error
            if (enable_distance && _nf.frame_available && _nf.dists_available) {
                    res_count = nodeframe_distance_residual(_nf, _poses, _residual, res_count);
            }

        }
        return true;
    }


    bool no_est_yaw_init_mode = false;

    SwarmFrameError(const SwarmFrame &_sf, 
                    const std::map<int, int> &_id2poseindex, 
                    const std::map<int, bool> & _yaw_observability, 
                    const std::map<int, double> & _yaw_init = std::map<int, double> (),
                    bool _detection_no_scale = false,
                    bool _enable_distance = false) :
            sf(_sf),
            id2poseindex(_id2poseindex),
            yaw_observability(_yaw_observability),
            yaw_init(_yaw_init),
            detection_no_scale(_detection_no_scale),
            enable_distance(_enable_distance)
    {}
};

typedef ceres::DynamicAutoDiffCostFunction<SwarmFrameError, AUTODIFF_STRIDE>  SFErrorCost;
// typedef ceres::DynamicAutoDiffCostFunction<SwarmDetectionError, AUTODIFF_STRIDE> DetectionCost;