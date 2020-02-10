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
#include "swarm_types.hpp"
#include <ros/ros.h>

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace Swarm;
using namespace Eigen;

typedef std::vector<Vector3d> vec_array;
typedef std::vector<Quaterniond> quat_array;

#define NO_ANNETAPOS



#define DETECTION_COV_POS 1

#define DETECTION_COV_ANG 1
#define ENABLE_DETECTION
// #define INIT_FXIED_YAW
#define ENABLE_LOOP

// Pose in this file use only x, y, z, yaw
//                            0  1  2   3


//idstamppose[id][stamp] -> pose index in poses
typedef std::map<int64_t, std::map<int, int>> IDStampPose;


template<typename T>
inline void pose_error(const T *posea, const T *poseb, T *error,
                       Eigen::Vector3d pos_cov = Eigen::Vector3d(0.01, 0.01, 0.01),
                       double ang_cov = 0.01) {
    error[0] = ERROR_NORMLIZED*(posea[0] - poseb[0]) / pos_cov.x();
    error[1] = ERROR_NORMLIZED*(posea[1] - poseb[1]) / pos_cov.y();
    error[2] = ERROR_NORMLIZED*(posea[2] - poseb[2]) / pos_cov.z();
    error[3] = ERROR_NORMLIZED*wrap_angle(poseb[3] - posea[3]) / ang_cov;
}

template<typename T>
inline void position_error(const T *posea, const T *poseb, T *error,
                       Eigen::Vector3d pos_cov = Eigen::Vector3d(0.01, 0.01, 0.01)) {
    error[0] = ERROR_NORMLIZED*(posea[0] - poseb[0]) / pos_cov.x();
    error[1] = ERROR_NORMLIZED*(posea[1] - poseb[1]) / pos_cov.y();
    error[2] = ERROR_NORMLIZED*(posea[2] - poseb[2]) / pos_cov.z();
}


//TODO: Add direction to this
template<typename T>
inline void unit_position_error(const T *posea, const T *poseb, const T * tangent_base, T *error,
                       Eigen::Vector3d pos_cov = Eigen::Vector3d(0.002, 0.002, 0.002)) {
    //For this residual; we assume poseb a unit vector
    const T scalea = sqrt(posea[0]*posea[0] +  posea[1]*posea[1] +  posea[2]*posea[2]);
    const T err0 = ERROR_NORMLIZED*(posea[0]/scalea - poseb[0]) / pos_cov.x();
    const T err1 = ERROR_NORMLIZED*(posea[1]/scalea - poseb[1]) / pos_cov.y();
    const T err2 = ERROR_NORMLIZED*(posea[2]/scalea - poseb[2]) / pos_cov.z();

    error[0] = tangent_base[0] * err0 + tangent_base[1] * err1 + tangent_base[2] * err2;
    error[1] = tangent_base[3] * err0 + tangent_base[4] * err1 + tangent_base[5] * err2;
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


struct SwarmLoopError {
    std::vector<Swarm::LoopConnection> locs;
    std::map<int, std::map<int64_t, int>> id_ts_poseindex;

    SwarmLoopError(std::vector<Swarm::LoopConnection> _locs, std::map<int, std::map<int64_t, int>>  _id_ts_poseindex) :
        locs(_locs), id_ts_poseindex(_id_ts_poseindex) {

    }

    template<typename T>
    inline void get_pose(int _id, int64_t ts, T const *const *_poses, T * t_pose) const {
        if (has_id_ts(_id, ts)) {
                int index = id_ts_poseindex.at(_id).at(ts);
                t_pose[0] =  _poses[index][0];
                t_pose[1] =  _poses[index][1];
                t_pose[2] =  _poses[index][2];
                t_pose[3] =  _poses[index][3];
        } else {
            ROS_ERROR("No pose of ID %d ts %d in Loop error;exit;", _id, TSShort(ts));
            exit(-1);
        }
    }

    template<typename T>
    inline void estimate_relpose(int ida, int64_t tsa, int idb, int64_t tsb, T const *const *_poses, T *relpose) const {
        T posea[4] , poseb[4];
        get_pose(ida, tsa, _poses, posea);
        get_pose(idb, tsb, _poses, poseb);
        DeltaPose(posea, poseb, relpose);
    }

    bool has_id_ts(int _id, int64_t ts) const {
        if (id_ts_poseindex.find(_id) != id_ts_poseindex.end()) {
            if(id_ts_poseindex.at(_id).find(ts) != id_ts_poseindex.at(_id).end()) {
                return true;
            }
        }
        return false;
    }

    template<typename T>
    inline int loop_relpose_residual(const Swarm::LoopConnection & loc, T const *const *_poses, T *_residual, int res_count) const {
        int _ida = loc.id_a;
        int _idb = loc.id_b;
        int64_t _tsa = loc.ts_a;
        int64_t _tsb = loc.ts_b;

        if (has_id_ts(_ida, _tsa) && has_id_ts(_idb, _tsb)) {
            Pose _rel_pose = loc.relative_pose;
            T rel_pose[4];
            _rel_pose.to_vector_xyzyaw(rel_pose);

            T relpose_est[4];
            estimate_relpose(_ida, _tsa, _idb, _tsb, _poses, relpose_est);

            pose_error(relpose_est, rel_pose, _residual + res_count, Eigen::Vector3d::Ones() * LOOP_COV,LOOP_YAWCOV);
            res_count = res_count + 4;
        }
        return res_count;
    }

    int residual_count() {
        return locs.size()*4;
    }

    template<typename T>
    bool operator()(T const *const *_poses, T *_residual) const {
        int res_count = 0;
        for (auto & loc : locs) {
            res_count = loop_relpose_residual(loc, _poses, _residual, res_count);
        }
        return true;
    }
};

struct SwarmFrameError {
    SwarmFrame sf;
    std::map<int, int> id2poseindex;
    std::map<int, bool> yaw_observability;
    std::map<int, double> yaw_init;
    bool detection_no_scale = false;

    template<typename T>
    inline void get_pose(int _id, T const *const *_poses, T * t_pose) const {
//        printf("%d", _id);
        if (id2poseindex.find(_id) != id2poseindex.end()) {
            int index = id2poseindex.at(_id);
            t_pose[0] =  _poses[index][0];
            t_pose[1] =  _poses[index][1];
            t_pose[2] =  _poses[index][2];
            if (yaw_observability.at(_id)) {
                t_pose[3] =  _poses[index][3];
            } else {
                t_pose[3] = T(yaw_init.at(_id));
            }

        } else {
            ROS_ERROR("No pose of ID %d in SF %d error;exit; SF Has only %ld id ", _id, TSShort(sf.ts), sf.id2nodeframe.size());
            for (auto it : id2poseindex) {
                ROS_ERROR("id %d", it.first);
            }
            exit(-1);
        }
    }

    template<typename T>
    inline void estimate_relpose(int ida, int idb, T const *const *_poses, T *relpose) const {
        T posea[4] , poseb[4];
        get_pose(ida, _poses, posea);
        get_pose(idb, _poses, poseb);
        DeltaPose(posea, poseb, relpose);
    }

    //Need add anntena position here!
    template<typename T>
    inline T node_distance(int idi, int idj, T const *const *_poses) const {
        //If consider bias here?
        T posea[4] , poseb[4];
        get_pose(idi, _poses, posea);
        get_pose(idj, _poses, poseb);



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
            if (has_id(_idj)  && _nf.enabled_distance.at(_idj)) {
                T _dis = T(it.second);
                //Less accuracy on distance
                _residual[res_count] = (node_distance(_nf.id, _idj, _poses) - _dis) *ERROR_NORMLIZED/ DISTANCE_MEASURE_ERROR;
                res_count++;
            }
        }
        return res_count;
    }

    template<typename T>
    inline int nodeframe_relpose_residual(NodeFrame &_nf, T const *const *_poses, T *_residual, int res_count) const {
        for (const auto & it: _nf.detected_nodes) {
//            Detected pose error
            int _id = it.first;
            if (has_id(_id) && _nf.enabled_detection.at(it.first)) {
                Pose _rel_pose = it.second;
                T rel_pose[4];
                _rel_pose.to_vector_xyzyaw(rel_pose);

                T relpose_est[4];
                estimate_relpose(_nf.id, _id, _poses, relpose_est);
                
                if(detection_no_scale) {
                    T tan_base[6];
                    auto _tan_base = _nf.detect_tan_base[_id];
                    EigenTanbase2T(_tan_base, tan_base);
                    unit_position_error(relpose_est, rel_pose, tan_base, _residual + res_count);
                    res_count = res_count + 2;
                } else {
                    Eigen::Vector3d pos_cov = _nf.detected_nodes_posvar[_id] * DETECTION_COV_POS;
                    position_error(relpose_est, rel_pose, _residual + res_count, pos_cov);
                    res_count = res_count + 3;
                }
            
            }
        }
        // ROS_INFO("Work with detected node");
        return res_count;
    }

    int residual_count() {
        int res_count = 0;
        for (const auto & it : sf.id2nodeframe) {
//            auto _id = it.first;
            const NodeFrame &_nf = it.second;

            //First we come to distance error
            if (_nf.frame_available) {

                if (_nf.dists_available) {
                    // ROS_WARN("TS %d ID %d ENABLED %ld DISMAP %ld\n", TSShort(_nf.ts), _nf.id, _nf.dis_map.size(), _nf.enabled_distance.size());
                    for (auto it : _nf.dis_map) {
                        if (has_id(it.first) && _nf.enabled_distance.at(it.first))
                            res_count++;
                    }
                }
                if (_nf.has_detect_relpose) {
                    for (const auto & it: _nf.detected_nodes) {
                        if (has_id(it.first) && _nf.enabled_detection.at(it.first)) {
                            if(detection_no_scale) {
                                res_count = res_count + 2;
                            } else{
                                res_count = res_count + 3;
                            }
                        }
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
                if (_nf.has_detect_relpose) {
                    res_count = nodeframe_relpose_residual(_nf, _poses, _residual, res_count);
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
    std::vector<Eigen::Vector3d> delta_pose_covs;
    std::vector<double> delta_ang_covs;
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
            delta_pose_covs.push_back(_nf_win[i].position_cov_to_last);
            delta_ang_covs.push_back(_nf_win[i].yaw_cov_to_last);
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

    Eigen::Vector3d pos_cov = Eigen::Vector3d::Ones() * VO_DRIFT_METER;
    double ang_cov = VO_ERROR_ANGLE;

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


            pose_error(est_dpose, mea_dpose, _residual + res_count, delta_pose_covs[i], delta_ang_covs[i]);


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