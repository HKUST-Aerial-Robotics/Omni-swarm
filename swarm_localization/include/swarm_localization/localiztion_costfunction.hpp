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

// #define DynamicCovarianceScaling
#define DCS_PHI ((T)(1))
// Pose in this file use only x, y, z, yaw
//                            0  1  2   3

//idstamppose[id][stamp] -> pose index in poses
typedef std::map<int64_t, std::map<int, int>> IDStampPose;


template<typename T>
inline void pose_error(const T *posea, const T *poseb, T *error,
                       Eigen::Vector3d pos_std = Eigen::Vector3d(0.01, 0.01, 0.01),
                       double ang_std = 0.01) {
    error[0] = ERROR_NORMLIZED*(posea[0] - poseb[0]) / pos_std.x();
    error[1] = ERROR_NORMLIZED*(posea[1] - poseb[1]) / pos_std.y();
    error[2] = ERROR_NORMLIZED*(posea[2] - poseb[2]) / pos_std.z();
    error[3] = ERROR_NORMLIZED*wrap_angle(poseb[3] - posea[3]) / ang_std;
#ifdef DynamicCovarianceScaling
    T X2 = error[0]*error[0] + error[1]*error[1] + error[2]*error[2] + error[3]*error[3];
    T s = sqrt((T)2*DCS_PHI/(DCS_PHI + X2));
    if (s < (T)1) {
        error[0] = s * error[0];
        error[1] = s * error[1];
        error[2] = s * error[2];
        error[3] = s * error[3];
    }
#endif
}

template<typename T>
inline void position_error(const T *posea, const T *poseb, T *error,
                       Eigen::Vector3d pos_std = Eigen::Vector3d(0.01, 0.01, 0.01)) {
    error[0] = ERROR_NORMLIZED*(posea[0] - poseb[0]) / pos_std.x();
    error[1] = ERROR_NORMLIZED*(posea[1] - poseb[1]) / pos_std.y();
    error[2] = ERROR_NORMLIZED*(posea[2] - poseb[2]) / pos_std.z();
}


template<typename T>
inline void unit_position_error(const T *posea, const T *poseb, const double * tangent_base, T *error) {
    //For this residual; we assume poseb a unit vector
    const T _inv_dep = 1.0/sqrt(posea[0]*posea[0] +  posea[1]*posea[1] +  posea[2]*posea[2]);
    const T err0 = ERROR_NORMLIZED*(posea[0]*_inv_dep - poseb[0]);
    const T err1 = ERROR_NORMLIZED*(posea[1]*_inv_dep - poseb[1]);
    const T err2 = ERROR_NORMLIZED*(posea[2]*_inv_dep - poseb[2]);

    error[0] = (tangent_base[0] * err0 + tangent_base[1] * err1 + tangent_base[2] * err2) / (T)(DETECTION_SPHERE_STD);
    error[1] = (tangent_base[3] * err0 + tangent_base[4] * err1 + tangent_base[5] * err2) / (T)(DETECTION_SPHERE_STD);
}

template<typename T>
inline void unit_position_error_inv_dep(const T *posea, const T *poseb, const T inv_dep, const double * tangent_base, T *error) {
    //For this residual; we assume poseb a unit vector
    const T _inv_dep = 1.0/sqrt(posea[0]*posea[0] +  posea[1]*posea[1] +  posea[2]*posea[2]);
    const T err0 = ERROR_NORMLIZED*(posea[0]*_inv_dep - poseb[0]);
    const T err1 = ERROR_NORMLIZED*(posea[1]*_inv_dep - poseb[1]);
    const T err2 = ERROR_NORMLIZED*(posea[2]*_inv_dep - poseb[2]);

    error[0] = (tangent_base[0] * err0 + tangent_base[1] * err1 + tangent_base[2] * err2) / (T)(DETECTION_SPHERE_STD);
    error[1] = (tangent_base[3] * err0 + tangent_base[4] * err1 + tangent_base[5] * err2) / (T)(DETECTION_SPHERE_STD);
    error[2] = (inv_dep - _inv_dep)*ERROR_NORMLIZED / (T)(DETECTION_INV_DEP_STD);

    // std::cout << "Pa" << posea[0] << " " << posea[1] << " " << posea[2] << std::endl;
    // std::cout << "Pb" << poseb[0] << " " << poseb[1] << " " << poseb[2] << std::endl;
    // std::cout << "err" << err0 << " " << err1 << " " << err2 << std::endl;
    // std::cout << "Error" << error[0] << " " << error[1] << " " << error[2] << std::endl;
}

template<typename T>
inline void unit_position_error(const T *posea, const T *poseb, const T dep, const double * tangent_base, T *error) {
    //For this residual; we assume poseb a unit vector
    const T _dep = sqrt(posea[0]*posea[0] +  posea[1]*posea[1] +  posea[2]*posea[2]);
    const T err0 = ERROR_NORMLIZED*(posea[0]/_dep - poseb[0]);
    const T err1 = ERROR_NORMLIZED*(posea[1]/_dep - poseb[1]);
    const T err2 = ERROR_NORMLIZED*(posea[2]/_dep - poseb[2]);

    error[0] = (tangent_base[0] * err0 + tangent_base[1] * err1 + tangent_base[2] * err2) / (T)(DETECTION_SPHERE_STD);
    error[1] = (tangent_base[3] * err0 + tangent_base[4] * err1 + tangent_base[5] * err2) / (T)(DETECTION_SPHERE_STD);
    error[2] = (_dep - dep)*ERROR_NORMLIZED / (T)(DETECTION_DEP_STD);

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

class GeneralMeasurement2DronesError {
protected:
    const Swarm::GeneralMeasurement2Drones * loc;
    GeneralMeasurement2DronesError(const Swarm::GeneralMeasurement2Drones* _loc): 
    loc(_loc){

    }
    
    template<typename T>
    inline void get_pose_a(T const *const *_poses, T * t_pose) const {
        t_pose[0] =  _poses[0][0];
        t_pose[1] =  _poses[0][1];
        t_pose[2] =  _poses[0][2];
        t_pose[3] =  _poses[0][3];
    }

    template<typename T>
    inline void get_pose_b(T const *const *_poses, T * t_pose) const {
        t_pose[0] =  _poses[1][0];
        t_pose[1] =  _poses[1][1];
        t_pose[2] =  _poses[1][2];
        t_pose[3] =  _poses[1][3];
    }

    template<typename T>
    inline void estimate_relpose(T const *const *_poses, T *relpose) const {
        T posea[4] , poseb[4];
        get_pose_a(_poses, posea);
        get_pose_b(_poses, poseb);
        DeltaPose(posea, poseb, relpose);
    }

public:
    virtual int residual_count() {
        return 0;
    }
};
class SwarmLoopError : public GeneralMeasurement2DronesError {
    const Swarm::LoopConnection* loop;
public:
    Eigen::Vector3d loop_std;
    double yaw_std;
    SwarmLoopError(const Swarm::GeneralMeasurement2Drones* _loc) :
        GeneralMeasurement2DronesError(_loc){
        loop = static_cast<const Swarm::LoopConnection*>(loc);
        double loop_xy_std = LOOP_POS_STD_0 + LOOP_POS_STD_SLOPE * loop->relative_pose.pos().norm();
        double loop_yaw_std = LOOP_YAW_STD_0 + LOOP_YAW_STD_SLOPE * loop->relative_pose.pos().norm();
        loop_std = Eigen::Vector3d(loop_xy_std, loop_xy_std, loop_xy_std)/loop->avg_count;
        yaw_std = loop_yaw_std/loop->avg_count;
    }

    virtual int residual_count() override { 
        return 4;
    }

    template<typename T>
    bool operator()(T const *const *_poses, T *_residual) const {
        int res_count = loop_relpose_residual(_poses, _residual);
        // std::cout << "LOOP RES COUNT " << res_count << std::endl;
        return true;
    }

protected:
    template<typename T>
    inline int loop_relpose_residual(T const *const *_poses, T *_residual) const {
        Pose _rel_pose = loop->relative_pose;
        T rel_pose[4];
        _rel_pose.to_vector_xyzyaw(rel_pose);

        T relpose_est[4];
        estimate_relpose(_poses, relpose_est);
        pose_error(relpose_est, rel_pose, _residual, loop_std, yaw_std);
        return 4;
    }
};

class SwarmDetectionError : public GeneralMeasurement2DronesError{
    bool enable_depth;
    Swarm::DroneDetection det;
    bool enable_dpose;
    Eigen::Vector3d dir;
    double inv_dep;
    double dep;
    bool use_inv_dep = false;
public:
    SwarmDetectionError(const Swarm::GeneralMeasurement2Drones* _loc) :
        GeneralMeasurement2DronesError(_loc){
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

    virtual int residual_count() override{
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

struct SwarmFrameError {
    SwarmFrame sf;
    std::map<int, int> id2poseindex;
    std::map<int, bool> yaw_observability;
    std::map<int, double> yaw_init;
    bool detection_no_scale = false;

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
                _residual[res_count] = (node_distance(_nf.id, _idj, _poses) - _dis) / ((T)(DISTANCE_STD))*ERROR_NORMLIZED;
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

            if (_nf.frame_available) {

                if (_nf.dists_available) {
                    // ROS_WARN("TS %d ID %d ENABLED %ld DISMAP %ld\n", TSShort(_nf.ts), _nf.id, _nf.dis_map.size(), _nf.enabled_distance.size());
                    for (auto it : _nf.dis_map) {
                        if (has_id(it.first) && _nf.distance_available(it.first)) 
                            res_count++;
                    }
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
            if (_nf.frame_available) {

                if (_nf.dists_available) {
                    res_count = nodeframe_distance_residual(_nf, _poses, _residual, res_count);
                }
            }

        }
        return true;
    }


    bool no_est_yaw_init_mode = false;

    SwarmFrameError(const SwarmFrame &_sf, 
                    const std::map<int, int> &_id2poseindex, 
                    const std::map<int, bool> & _yaw_observability, 
                    const std::map<int, double> & _yaw_init = std::map<int, double> (),
                    bool _detection_no_scale = false) :
            sf(_sf),
            id2poseindex(_id2poseindex),
            yaw_observability(_yaw_observability),
            yaw_init(_yaw_init),
            detection_no_scale(_detection_no_scale){
    }
};

//Error for correlation vo drift
struct SwarmHorizonError {
    const std::vector<NodeFrame> nf_windows;
    std::map<int64_t, int> ts2nfindex;
    std::map<int64_t, int> ts2poseindex;
    bool yaw_observability;
    std::vector<double> yaw_init;

    std::vector<Pose> delta_poses;
    std::vector<Eigen::Vector3d> delta_pose_stds;
    std::vector<double> delta_ang_stds;
    int _id = -1;

    SwarmHorizonError(const std::vector<NodeFrame> &_nf_win, const std::map<int64_t, int> &_ts2poseindex, bool _yaw_observability, std::vector<double> _yaw_init) :
            nf_windows(_nf_win),
            ts2poseindex(_ts2poseindex),
            yaw_observability(_yaw_observability),
            yaw_init(_yaw_init){
        ts2nfindex[_nf_win[0].ts] = 0;
        for (unsigned int i = 1; i< _nf_win.size(); i++) {
            auto _nf = _nf_win[i];
            delta_poses.push_back(Pose::DeltaPose(_nf_win[i-1].pose(), _nf.pose(), true));
            delta_pose_stds.push_back(_nf_win[i].position_std_to_last);
            // std::cout << "Delta Pos"<< delta_poses.back().pos() << "Pose STD to last" << _nf_win[i].position_std_to_last << std::endl;
            delta_ang_stds.push_back(_nf_win[i].yaw_std_to_last);
            ts2nfindex[_nf.ts] = i;
        }

        _id = nf_windows.back().id;
    }

    template<typename T>
    inline void get_pose(int64_t ts, T const *const *_poses, T * t_pose) const {
        if (ts2poseindex.find(ts) != ts2poseindex.end()) {
            int index = ts2poseindex.at(ts);
            t_pose[0] =  _poses[index][0];
            t_pose[1] =  _poses[index][1];
            t_pose[2] =  _poses[index][2];
            if (yaw_observability) {
                t_pose[3] =  _poses[index][3];
            } else {
                t_pose[3] = T(yaw_init.at(ts2nfindex.at(ts)));
            }
        } else {
            ROS_ERROR("No pose of ID,%d TS %d in swarm horizon error;exit", _id, TSShort(ts));
            exit(-1);
        }
    }

    int residual_count() {
        return (nf_windows.size()-1)*4;
    }

    template<typename T>
    bool operator()(T const *const *_poses, T *_residual) const {

        int res_count = 0;
        for (unsigned int i = 0; i < nf_windows.size() - 1; i++) {
            //estimate deltapose
            Pose _mea_dpose = delta_poses[i]; // i to i + 1 Pose
            T mea_dpose[4], est_posea[4], est_poseb[4];

            _mea_dpose.to_vector_xyzyaw(mea_dpose);

            int64_t tsa = nf_windows[i].ts;
            int64_t tsb = nf_windows[i+1].ts;

            get_pose(tsa, _poses, est_posea);
            get_pose(tsb, _poses, est_poseb);

            T est_dpose[4];
            DeltaPose(est_posea, est_poseb, est_dpose);


            pose_error(est_dpose, mea_dpose, _residual + res_count, delta_pose_stds[i], delta_ang_stds[i]);


            /*
            if (nf_windows[0].id == 0 && i==0 ) {
                printf("ID %d i %d tsa %d", nf_windows[0].id, i, (tsa/1000000)%1000000);
                std::cout <<"residual"<< Eigen::Map<const Eigen::Matrix<T, 4, 1> >(est_dpose) << std::endl;
                std::cout <<"esta"<< Eigen::Map<const Eigen::Matrix<T, 4, 1> >(est_posea) << std::endl;
                std::cout <<"estb"<< Eigen::Map<const Eigen::Matrix<T, 4, 1> >(est_poseb) << std::endl;
                std::cout <<"mea dpose"<< Eigen::Map<const Eigen::Matrix<T, 4, 1> >(mea_dpose) << std::endl;
                std::cout <<"est dpose"<< Eigen::Map<const Eigen::Matrix<T, 4, 1> >(est_dpose) << "\n\n\n" << std::endl;
            }*/
            res_count = res_count + 4;

        }
        return true;
    }
};

#define AUTODIFF_STRIDE 4
typedef ceres::DynamicAutoDiffCostFunction<SwarmFrameError, AUTODIFF_STRIDE>  SFErrorCost;
typedef ceres::DynamicAutoDiffCostFunction<SwarmHorizonError, AUTODIFF_STRIDE> HorizonCost;
typedef ceres::DynamicAutoDiffCostFunction<SwarmLoopError, AUTODIFF_STRIDE> LoopCost;
typedef ceres::DynamicAutoDiffCostFunction<SwarmDetectionError, AUTODIFF_STRIDE> DetectionCost;