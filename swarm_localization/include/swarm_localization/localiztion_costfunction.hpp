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

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace swarm;
using namespace Eigen;

typedef std::vector<Vector3d> vec_array;
typedef std::vector<Quaterniond> quat_array;


#define VO_DRIFT_METER 0.1
#define VO_ERROR_ANGLE 0.001
#define DISTANCE_MEASURE_ERROR 1.0

// Pose in this file use only x, y, z, yaw
//                            0  1  2   3


//idstamppose[id][stamp] -> pose index in poses
typedef std::map<int64_t, std::map<int, int>> IDStampPose;


template<typename T>
inline void pose_error(const T *posea, const T *poseb, T *error,
                       Eigen::Vector3d pos_cov = Eigen::Vector3d(0.01, 0.01, 0.01),
                       double ang_cov = 0.01) {
    error[0] = (posea[0] - poseb[0]) / pos_cov.x();
    error[1] = (posea[1] - poseb[1]) / pos_cov.y();
    error[2] = (posea[2] - poseb[2]) / pos_cov.z();
    error[3] = wrap_angle(poseb[3] - posea[3]) / ang_cov;
}

template<typename T>
inline void YawRotatePoint(T yaw, const T * vec, T * ret) {
    ret[0] = cos(yaw) * vec[0] - sin(yaw)*vec[1];
    ret[1] = sin(yaw) * vec[0] + cos(yaw)*vec[1];
    ret[2] = vec[2];
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

struct SwarmFrameError {
    SwarmFrame sf;
    std::map<int, int> id2poseindex;
    int self_id = -1;
    bool is_lastest_frame = false;

    template<typename T>
    void get_pose(int _id, T const *const *_poses, T * t_pose) const {
//        printf("%d", _id);
        if (_id == self_id && is_lastest_frame) {

            //May have risk of returning this t_pose
            Pose _pose =  sf.id2nodeframe.at(_id).pose();
            _pose.to_vector_xyzyaw(t_pose);
            return;
        } else {
            int index = id2poseindex.at(_id);
            t_pose[0] =  _poses[index][0];
            t_pose[1] =  _poses[index][1];
            t_pose[2] =  _poses[index][2];
            t_pose[3] =  _poses[index][3];
        }

    }

    template<typename T>
    void estimate_relpose(int ida, int idb, T const *const *_poses, T *relpose) const {
        T posea[7] , poseb[7];
        get_pose(ida, _poses, posea);
        get_pose(idb, _poses, poseb);
        DeltaPose(posea, poseb, relpose);
    }

    template<typename T>
    T node_distance(int idi, int idj, T const *const *_poses) const {
        //If consider bias here?
        T posea[7] , poseb[7];
        get_pose(idi, _poses, posea);
        get_pose(idj, _poses, poseb);
        return sqrt((poseb[0] - posea[0]) * (poseb[0] - posea[0])
                    + (poseb[1] - posea[1]) * (poseb[1] - posea[1])
                    + (poseb[2] - posea[2]) * (poseb[2] - posea[2]));
    }


    template<typename T>
    int nodeframe_distance_res(NodeFrame &_nf, T const *const *_poses, T *_residual, int res_count) const {
        for (auto it : _nf.dis_map) {
            int _idj = it.first;
            T _dis = T(it.second);
            //Less accuracy on distance
            _residual[res_count] = (node_distance(_nf.id, _idj, _poses) - _dis) / DISTANCE_MEASURE_ERROR;
            res_count++;
        }
        return res_count;
    }

    template<typename T>
    int nodeframe_relpose_res(NodeFrame &_nf, T const *const *_poses, T *_residual, int res_count) const {
        for (auto it: _nf.detected_nodes) {
//            Detected pose error
            int _id = it.first;
            Pose _rel_pose = it.second;
            T rel_pose[4];
            _rel_pose.to_vector_xyzyaw(rel_pose);

            T relpose_est[4];
            estimate_relpose(_nf.id, _id, _poses, relpose_est);

            Eigen::Vector3d pos_cov = _nf.detected_nodes_poscov[_id];
            Eigen::Vector3d ang_cov = _nf.detected_nodes_angcov[_id];

            pose_error(relpose_est, rel_pose, _residual + res_count, pos_cov, ang_cov.z());
            res_count = res_count + 4;
        }
        return res_count;
    }

    int residual_count() {
        int res_count = 0;
        for (auto it : sf.id2nodeframe) {
//            auto _id = it.first;
            NodeFrame &_nf = it.second;

            //First we come to distance error
            if (_nf.frame_available) {

                if (_nf.dists_available) {
                    for (auto it : _nf.dis_map) {
                        res_count++;
                    }
                }

                if (_nf.has_detect_relpose) {
                    for (auto it: _nf.detected_nodes) {
                        res_count = res_count + 4;
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
                    res_count = nodeframe_distance_res(_nf, _poses, _residual, res_count);
                }

                if (_nf.has_detect_relpose) {
                    res_count = nodeframe_relpose_res(_nf, _poses, _residual, res_count);
                }

            }

        }

        return true;

    }


    SwarmFrameError(const SwarmFrame &_sf, const std::map<int, int> &_id2poseindex, bool _is_lastest_frame) :
            sf(std::move(_sf)),
            id2poseindex(std::move(_id2poseindex)),
            is_lastest_frame(_is_lastest_frame) {
            self_id = _sf.self_id;
    }


};


//Error for correlation vo drift
struct SwarmHorizonError {
    const std::vector<NodeFrame> nf_windows;
    std::map<int64_t, int> ts2poseindex;
    int64_t last_ts = -1;

    std::vector<Pose> delta_poses;

    bool is_self_node = false;
    SwarmHorizonError(const std::vector<NodeFrame> &_nf_win, const std::map<int64_t, int> &_ts2poseindex, bool _is_self_node) :
            nf_windows(_nf_win),
            ts2poseindex(_ts2poseindex),
            is_self_node(_is_self_node){

        for (unsigned int i = 1; i< _nf_win.size(); i++) {
            auto _nf = _nf_win[i];
//            std::cout << i << ":" << _nf_win[i-1].pose().position << std::endl;
            delta_poses.push_back(Pose::DeltaPose(_nf_win[i-1].pose(), _nf.pose(), true));
//            delta_poses.push_back(Pose());
//            printf("ID %d TS %ld DPOSE ", _nf.id, (_nf.ts/1000000)%1000000);
//            delta_poses.back().print();
//            printf("\n");
        }
        last_ts = nf_windows.back().ts;
    }

    template<typename T>
    void get_pose(int64_t ts, T const *const *_poses, T * t_pose) const {

        if (is_self_node && ts == last_ts) {
            Pose _pose = nf_windows.back().pose();
            _pose.to_vector_xyzyaw(t_pose);
            return;
        } else {
            int index = ts2poseindex.at(ts);
            t_pose[0] =  _poses[index][0];
            t_pose[1] =  _poses[index][1];
            t_pose[2] =  _poses[index][2];
            t_pose[3] =  _poses[index][3];
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

            //
            get_pose(tsa, _poses, est_posea);
            get_pose(tsb, _poses, est_poseb);


           Eigen::Vector3d pos_cov = Eigen::Vector3d(1, 1, 1) * VO_DRIFT_METER;
           Eigen::Vector3d ang_cov = Eigen::Vector3d(1, 1, 1) * VO_ERROR_ANGLE;

           T est_dpose[4];
           DeltaPose(est_posea, est_poseb, est_dpose);
           pose_error(est_dpose, mea_dpose, _residual + res_count,
                      pos_cov, ang_cov.z());

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