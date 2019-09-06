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


#define VO_DRIFT_METER 0.003 //1/100m; 2e-3 per kf
#define VO_DRIFT_METER_Z 0.005
#define VO_ERROR_ANGLE 3e-6 //3deg/1000m; average kf 0.2m, e.g 6e-4deg kf, eg 3e^-6
#define DISTANCE_MEASURE_ERROR 0.1
#define ERROR_NORMLIZED 0.01
//#define DETECTION_COV_POS 10
#define DETECTION_COV_POS 1
#define DISABLE_DETECTION_YAW

#define DETECTION_COV_ANG 1
#define ENABLE_DETECTION

//#define ENABLE_HISTORY_COV


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


struct SwarmFrameError {
    SwarmFrame sf;
    std::map<int, int> id2poseindex;
    std::map<int, bool> yaw_observability;
    std::map<int, double> yaw_init;
    int self_id = -1;
    bool is_lastest_frame = false;
    Pose self_pose;

    template<typename T>
    inline void get_pose(int _id, T const *const *_poses, T * t_pose) const {
//        printf("%d", _id);
        if (_id == self_id && is_lastest_frame) {
            self_pose.to_vector_xyzyaw(t_pose);
            return;
        } else {
            if (id2poseindex.find(_id) != id2poseindex.end()) {
                int index = id2poseindex.at(_id);
                t_pose[0] =  _poses[index][0];
                t_pose[1] =  _poses[index][1];

                if (first_init_mode) {
                    t_pose[2] = T(sf.id2nodeframe.at(_id).position().z());
                    t_pose[3] = T(sf.id2nodeframe.at(_id).yaw());
                } else {
                    t_pose[2] =  _poses[index][2];
                    if (yaw_observability.at(_id)) {
                        t_pose[3] =  _poses[index][3];
                    } else {
                        t_pose[3] = T(yaw_init.at(_id));
                    }
                }

            } else {
                ROS_ERROR("No pose of ID %d in SF %d error;exit; SF Has only %ld id ", _id, TSShort(sf.ts), sf.id2nodeframe.size());
                for (auto it : id2poseindex) {
                    ROS_ERROR("id %d", it.first);
                }
                exit(-1);
            }
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



#ifdef NO_ANNETAPOS
        return sqrt((poseb[0] - posea[0]) * (poseb[0] - posea[0])
                    + (poseb[1] - posea[1]) * (poseb[1] - posea[1])
                    + (poseb[2] - posea[2]) * (poseb[2] - posea[2]));
#else
        T pa[3], pb[3];
        T p_anna[3], p_annb[3];
        EigenVec2T(sf.id2nodeframe.at(idi).get_anntena_pos(), p_anna);
        EigenVec2T(sf.id2nodeframe.at(idj).get_anntena_pos(), p_annb);
        PoseTransformPoint(posea, p_anna, pa);
        PoseTransformPoint(poseb, p_annb, pb);
        return sqrt((pb[0] - pa[0]) * (pb[0] - pa[0])
                    + (pb[1] - pa[1]) * (pb[1] - pa[1])
                    + (pb[2] - pa[2]) * (pb[2] - pa[2]));
#endif

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

                Eigen::Vector3d pos_cov = _nf.detected_nodes_posvar[_id] * DETECTION_COV_POS;

#ifdef DISABLE_DETECTION_YAW
                position_error(relpose_est, rel_pose, _residual + res_count, pos_cov);
                res_count = res_count + 3;
#else
                Eigen::Vector3d ang_cov = _nf.detected_nodes_angvar[_id] * DETECTION_COV_ANG;
                pose_error(relpose_est, rel_pose, _residual + res_count, pos_cov, ang_cov.z());
                res_count = res_count + 4;
#endif
            }
        }
        // ROS_INFO("Work with detected node");
        return res_count;
    }

    template<typename T>
    inline int nodeframe_first_init_align_residual(NodeFrame &_nf, T const *const *_poses, T *_residual, int res_count) const {
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
#ifdef ENABLE_DETECTION
                if (_nf.has_detect_relpose) {
                    for (const auto & it: _nf.detected_nodes) {
                        if (has_id(it.first) && _nf.enabled_detection.at(it.first)) {
#ifdef DISABLE_DETECTION_YAW
                            res_count = res_count + 3;
#else
                            res_count = res_count + 4;
#endif
                        }
                    }
                }
#endif
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
#ifdef ENABLE_DETECTION
                if (_nf.has_detect_relpose) {
                    res_count = nodeframe_relpose_residual(_nf, _poses, _residual, res_count);
                }
#endif          
                /*
                if (first_init_mode && _nf.id != self_id) {
                    res_count = nodeframe_first_init_align_residual(_nf, _poses, _residual, res_count);
                } */

            }

        }

        return true;

    }


    bool first_init_mode = false;

    SwarmFrameError(const SwarmFrame &_sf, 
                    const std::map<int, int> &_id2poseindex, 
                    const std::map<int, bool> & _yaw_observability, 
                    const std::map<int, double> & _yaw_init,
                    bool _is_lastest_frame, 
                    bool _first_init_mode = false) :
            sf(_sf),
            id2poseindex(_id2poseindex),
            yaw_observability(_yaw_observability),
            yaw_init(_yaw_init),
            is_lastest_frame(_is_lastest_frame),
            first_init_mode(_first_init_mode) {
            self_id = _sf.self_id;
            self_pose = sf.id2nodeframe.at(self_id).pose();
    }
};

struct SolvedPosewithCov{
    double pose[4]  = {0}; //xyzyaw
    Eigen::Vector3d pos_cov = Eigen::Vector3d(0, 0, 0);
    double yaw_cov = 0;
    SolvedPosewithCov () {

    }
};

//Error for correlation vo drift
struct SwarmHorizonError {
    const std::vector<NodeFrame> nf_windows;
    std::map<int64_t, int> ts2nfindex;
    std::map<int64_t, int> ts2poseindex;
    bool yaw_observability;
    std::vector<double> yaw_init;
    int64_t last_ts = -1;
    int64_t first_ts = -1;

    std::vector<Pose> delta_poses;
    int _id = -1;
    bool is_self_node = false;
    bool first_init_mode;

    SwarmHorizonError(const std::vector<NodeFrame> &_nf_win, const std::map<int64_t, int> &_ts2poseindex, bool _yaw_observability, std::vector<double> _yaw_init, bool _is_self_node, bool _first_init_mode) :
            nf_windows(_nf_win),
            ts2poseindex(_ts2poseindex),
            yaw_observability(_yaw_observability),
            yaw_init(_yaw_init),
            is_self_node(_is_self_node),
            first_init_mode(_first_init_mode){

        ts2nfindex[_nf_win[0].ts] = 0;
        for (unsigned int i = 1; i< _nf_win.size(); i++) {
            auto _nf = _nf_win[i];
            delta_poses.push_back(Pose::DeltaPose(_nf_win[i-1].pose(), _nf.pose(), true));
            ts2nfindex[_nf.ts] = i;
        }

        last_ts = nf_windows.back().ts;
        first_ts = nf_windows.front().ts;
        _id = nf_windows.back().id;
    }

    template<typename T>
    inline void get_pose(int64_t ts, T const *const *_poses, T * t_pose) const {

        if (is_self_node && ts == last_ts) {
            Pose _pose = nf_windows.back().pose();
            _pose.to_vector_xyzyaw(t_pose);
            return;
        }

        if (ts2poseindex.find(ts) != ts2poseindex.end()) {
            int index = ts2poseindex.at(ts);
            t_pose[0] =  _poses[index][0];
            t_pose[1] =  _poses[index][1];
            if (first_init_mode) {
                const NodeFrame & _nf = nf_windows.at(ts2nfindex.at(ts));
                t_pose[2] = T(_nf.position().z());
                t_pose[3] = T(_nf.yaw());
            } else {
                t_pose[2] =  _poses[index][2];
                if (yaw_observability) {
                    t_pose[3] =  _poses[index][3];
                } else {
                    t_pose[3] = T(yaw_init.at(ts2nfindex.at(ts)));
                }
            }
        } else {
            ROS_ERROR("No pose of ID,%d TS %d in swarm horizon error;exit", _id, TSShort(ts));
            exit(-1);
        }
    }

    int residual_count() {
        return (nf_windows.size()-1)*4;
    }

    Eigen::Vector3d pos_cov = Eigen::Vector3d(VO_DRIFT_METER, VO_DRIFT_METER, VO_DRIFT_METER_Z);
    Eigen::Vector3d ang_cov = Eigen::Vector3d(1, 1, 1) * VO_ERROR_ANGLE;

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


            pose_error(est_dpose, mea_dpose, _residual + res_count, pos_cov, ang_cov.z());


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