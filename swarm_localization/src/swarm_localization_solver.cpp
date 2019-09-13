//
// Created by xuhao on 19-6-4.
//

#include "swarm_localization/swarm_localization_solver.hpp"


#include <iostream>
#include "glog/logging.h"
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include <time.h>
#include <thread>
#include <unistd.h>
#include "swarm_localization/localiztion_costfunction.hpp"
#include <functional>
#include <swarm_localization/swarm_types.hpp>
#include <set>
#include <chrono>

using namespace std::chrono;

#define DEBUG_OUTPUT_POSES
//#define COMPUTE_COV
#define SMALL_MOVEMENT_SPD 0.1
#define REPLACE_MIN_DURATION 0.1
#define ENABLE_REPLACE
#define MAX_SOLVER_TIME 0.5
//#define MAX_SOLVER_TIME 1.0
// #define DEBUG_OUTPUT_COV
// #define ENABLE_HISTORY_COV
#define INIT_FXIED_YAW
#define INIT_Z_ERROR 0.05
#define NOT_MOVING_THRES 0.02
#define NOT_MOVING_YAW 0.05

#define THRES_YAW_OBSER_XY 1.0

#define INIT_TRIAL 5


bool SwarmLocalizationSolver::detect_outlier(const SwarmFrame &sf) const {
    //Detect if it's outlier

    // for (int i = 0; i  < s)
    return false;
}


int SwarmLocalizationSolver::judge_is_key_frame(const SwarmFrame &sf) {
    auto _ids = sf.node_id_list;
    std::vector<int> ret(0);
    if (_ids.size() < 2)
        return 0;

    if (sf_sld_win.empty()) {
        for (auto _id : _ids) {
            node_kf_count[_id] = 1;
        }

        if (sf.HasID(self_id) && sf.has_odometry(self_id)) {
            return 1;
        } else {
            return 0;
        }
    }

    const SwarmFrame & last_sf = sf_sld_win.back();
    double dt = (sf.stamp - last_sf.stamp).toSec();

    if (!sf.HasID(self_id) || !sf.has_odometry(self_id)) {
        return 0;
    }

    const NodeFrame & self_nf = sf.id2nodeframe.at(self_id);
    Eigen::Vector3d _diff = sf.position(self_id) - last_sf.position(self_id);

    //TODO: make it set to if last dont's have some detection and this frame has, than keyframe
    if (_diff.norm() > min_accept_keyframe_movement || 
        (_diff.norm() > min_accept_keyframe_movement*0.5 && sf.has_detect() > 0)   ){ //here shall be some one see him or he see someone
        ret.push_back(self_id);
        node_kf_count[self_id] += 1;
        ROS_INFO("SF %d is kf of %d: DIFF %3.2f HAS %d", 
            TSShort(sf.ts), self_id, _diff.norm(), sf.HasDetect(self_id));

        return 1;
    }

    for (auto it : sf.id2nodeframe) {
        //For other nodes
        int _id = it.first;
        const NodeFrame & _nf = it.second;
        if (_nf.is_valid && _nf.vo_available && last_sf.HasID(_id) && last_sf.has_odometry(_id)) {
            Eigen::Vector3d _diff = sf.position(_id) - last_sf.position(_id);
            if (_diff.norm() > min_accept_keyframe_movement*0.5 &&
            (self_nf.has_detected_node(_id) ) ) { //Because others watch itself don't provide yaw information; but this drone watch other gives yaw information
                ret.push_back(self_id);
                node_kf_count[self_id] += 1;
                ROS_INFO("SF %d is kf of %d: DIFF %3.2f HAS %d", 
                    TSShort(sf.ts), self_id, _diff.norm(), sf.HasDetect(self_id));
                return 1;
            }
        }
    }

    if (sf.swarm_size() >= last_sf.swarm_size() && _diff.norm() < SMALL_MOVEMENT_SPD * dt && sf.swarm_size() > last_sf.swarm_size() &&
    ( (dt > REPLACE_MIN_DURATION && sf.has_detect() == last_sf.has_detect()) || sf.has_detect() > last_sf.has_detect())
    ) {
        //Make sure is fixed
        return 2;
    }

    return 0;
}

void SwarmLocalizationSolver::delete_frame_i(int i) {
    sf_sld_win.erase(sf_sld_win.begin() + i);
}

bool SwarmLocalizationSolver::is_frame_useful(unsigned int i) const {
    for (unsigned int id : sf_sld_win.at(i).node_id_list) {
        if (node_kf_count.at(id) < min_frame_number) {
            return true;
        }
    }
    return false;
}

void SwarmLocalizationSolver::process_frame_clear() {
    //Delete non keyframe first

    /*
    while (i < sf_sld_win.size() && sf_sld_win.size() > max_frame_number) {
        if (!is_frame_useful(i)) {
            delete_frame_i(i);
        } else {
            i++;
        }
    } */

    while (sf_sld_win.size() > max_frame_number) {
        ROS_INFO("Start clear frames");
        SwarmFrame & sf = sf_sld_win[0];
        for (auto it : sf.id2nodeframe) {
            if (est_cov_tsid.find(sf.ts) != est_cov_tsid.end() && sf.has_odometry(it.first)) {
                last_lost_ts_of_node[it.first] = sf.ts;
            }
        }

        delete_frame_i(0);
        ROS_INFO("Clear first frame from sld win, now size %ld", sf_sld_win.size());
    }
}

void SwarmLocalizationSolver::random_init_pose(EstimatePoses &swarm_est_poses, EstimatePosesIDTS &est_poses_idts) {
    for (auto it : swarm_est_poses) {
        for (auto it2 : it.second) {
            if (it2.first != self_id) {
                double * p = it2.second;
                p[0] = rand_FloatRange(-10, 10);
                p[1] = rand_FloatRange(-10, 10);
                // p[2] = rand_FloatRange(-INIT_Z_ERROR, INIT_Z_ERROR);
                p[2] = all_sf[it.first].id2nodeframe[it2.first].position().z();
    #ifdef INIT_FXIED_YAW
                p[3] = all_sf[it.first].id2nodeframe[it2.first].yaw();
    #elif
                p[3] = rand_FloatRange(-M_PI, M_PI);
    #endif
            }
        }
    }
}

void SwarmLocalizationSolver::init_dynamic_nf_in_keyframe(int64_t ts, NodeFrame &_nf) {
    int _id = _nf.id;
    EstimatePoses & est_poses = est_poses_tsid;
    EstimatePosesIDTS & est_poses2 = est_poses_idts;
    auto _p = new double[4];
    if (_id != self_id) {
        double noise = this->initial_random_noise;

        Pose _last;
        if (last_kf_ts > 0 && est_poses_idts.find(_id) != est_poses_idts.end()) {
            //Use last solve relative res, e.g init with last

            int64_t last_ts_4node = est_poses_idts[_id].rbegin()->first;
            _last = Pose(est_poses_tsid[last_ts_4node][_id], true);

            Pose last_vo = all_sf[last_ts_4node].id2nodeframe[_id].pose();
            Pose now_vo = _nf.pose();
            now_vo.set_yaw_only();
            last_vo.set_yaw_only();

            Eigen::Isometry3d TnowVO = now_vo.to_isometry();
            Eigen::Isometry3d TlastVO = last_vo.to_isometry();

            Eigen::Isometry3d Tlast = _last.to_isometry();
            Eigen::Isometry3d dpose = TlastVO.inverse()*TnowVO;
            Pose dpose_(dpose);

            if ( dpose_.pos().norm() < NOT_MOVING_THRES && fabs(dpose_.yaw()) < NOT_MOVING_YAW ) {
                //NOT MOVING; Merging pose
                delete _p;
                _p = est_poses_tsid[last_ts_4node][_id];
            } else {
                Pose transfered_now(Tlast*dpose);
                transfered_now.to_vector_xyzyaw(_p);
            }
            // ROS_INFO("Init ID %d at %d with predict value", _nf.id, TSShort(ts));
        } else {
            ROS_INFO("Init ID %d at %d with random value", _nf.id, TSShort(ts));
            _last.set_pos(_nf.pose().pos() + rand_FloatRange_vec(-noise, noise));
            _last.set_att(_nf.pose().att());

            _last.to_vector_xyzyaw(_p);
        }
    } else {
        Pose p = _nf.pose();
        p.to_vector_xyzyaw(_p);
        // ROS_INFO("x y z yaw:%7.6f %7.6f %7.6f Y %7.6f", _p[0], _p[1], _p[2], _p[3]*57.3, noise);
    }

    if (est_poses.find(ts) == est_poses.end()) {
        est_poses[ts] = std::map<int, double*>();
    }

    if(est_poses2.find(_id) == est_poses2.end()) {
        est_poses2[_id] = std::map<int64_t, double*>();
    }
    est_poses[ts][_id] = _p;
    est_poses2[_id][ts] = _p;
}


void SwarmLocalizationSolver::init_static_nf_in_keyframe(int64_t ts, NodeFrame &_nf) {
    int _id = _nf.id;
    EstimatePoses & est_poses = est_poses_tsid;
    EstimatePosesIDTS & est_poses2 = est_poses_idts;
    double * _p = nullptr;
    if (last_kf_ts > 0 && est_poses2.find(_id) != est_poses2.end()) {
        _p = est_poses2[_id].begin()->second;
    } else {
        _p = new double[4];
        Pose _last;
        double noise = this->initial_random_noise;
        _last.set_pos(_nf.pose().pos() + rand_FloatRange_vec(-noise, noise));
        _last.set_att(_nf.pose().att());
        _last.to_vector_xyzyaw(_p);
    }

    if (est_poses.find(ts) == est_poses.end()) {
        est_poses[ts] = std::map<int, double*>();
    }

    if(est_poses2.find(_id) == est_poses2.end()) {
        est_poses2[_id] = std::map<int64_t, double*>();
    }
    est_poses[ts][_id] = _p;
    est_poses2[_id][ts] = _p;
}

void SwarmLocalizationSolver::print_frame(const SwarmFrame & sf) const {
    ROS_INFO("KF %d details\n", TSShort(sf.ts));
    if (!finish_init) {
        return;
    }
    const SwarmFrame & last_sf = all_sf.at(last_kf_ts);

    for (auto it : sf.id2nodeframe) {
        auto id = it.first;
        auto _nf = it.second;
        ROS_INFO("\n\nID %d ", id);
        if (est_poses_idts.at(id).find(last_kf_ts) == est_poses_idts.at(id).end() ) {
            ROS_INFO("Can't find id in last KF %d", TSShort(last_kf_ts));
            continue;
        }
        double* pose_last = est_poses_idts.at(id).at(last_kf_ts);
        if (!last_sf.HasID(id) || !last_sf.id2nodeframe.at(id).vo_available) 
            return;
        Pose pose_vo_last = last_sf .id2nodeframe.at(id).pose();
        int64_t ts =  sf.ts;
        double * pose = est_poses_idts.at(id).at(ts);
        auto pose_vo = sf.id2nodeframe.at(id).pose();
        auto poseest = Pose(pose, true);
        ROS_INFO("POSVO        %3.4f %3.4f %3.4f YAW %5.4fdeg",
                pose_vo.pos().x(), pose_vo.pos().y(), pose_vo.pos().z(), pose_vo.yaw()*57.3);
        ROS_INFO("POSEST     %3.4f %3.4f %3.4f YAW %5.4fdeg",
                poseest.pos().x(), poseest.pos().y(), poseest.pos().z(), pose_vo.yaw()*57.3);
        Pose DposeVO = Pose::DeltaPose(pose_vo_last, pose_vo, true);
        Pose DposeEST = Pose::DeltaPose(Pose(pose_last, true), Pose(pose, true), true);
        Pose ERRVOEST = Pose::DeltaPose(DposeVO, DposeEST, true);
        double ang_err = ERRVOEST.yaw()*1000;
        
        ROS_WARN("ERRVOEST(mm)       %6.5f %6.5f %6.5f ANG  %3.2f",
                ERRVOEST.pos().x()*1000, ERRVOEST.pos().y()*1000, ERRVOEST.pos().z()*1000, ang_err);

        ROS_INFO("DPOSVO         %6.5f %6.5f %3.4f YAW %5.4fdeg",
                DposeVO.pos().x(), DposeVO.pos().y(), DposeVO.pos().z(), DposeVO.yaw()*57.3);

        ROS_INFO("DPOSEST        %6.5f %6.5f %3.4f YAW %5.4fdeg\n\n",
                DposeEST.pos().x(), DposeEST.pos().y(), DposeEST.pos().z(), DposeEST.yaw()*57.3);

        printf("DISTANCES ");
        for (auto itj : _nf.dis_map) {
            int _idj = itj.first;
            double dis = itj.second;
            if (sf.HasID(_idj) && sf.id2nodeframe.at(_idj).vo_available) {
                if (est_poses_idts.find(_idj) == est_poses_idts.end() || est_poses_idts.at(_idj).find(ts) == est_poses_idts.at(_idj).end()) {
                    ROS_INFO("Can't find %d at %d", _idj, TSShort(ts));
                    continue;
                }

                Pose posj_est(est_poses_idts.at(_idj).at(ts), true);
                double est_dis = (posj_est.pos() - poseest.pos()).norm();
                printf("ID %d DIS %4.2f EST %4.2f ",_idj, dis, est_dis);
            }

        }

        printf("DETETCTIONS ");
        for (auto itj : _nf.detected_nodes) {
            int _idj = itj.first;
            Pose det = itj.second;
            if (sf.HasID(_idj) && sf.id2nodeframe.at(_idj).vo_available) {
                Pose posj_est(est_poses_idts.at(_idj).at(ts), true);
                Pose DposeEST = Pose::DeltaPose(posj_est, _nf.pose(), true);
                double est_dis = (posj_est.pos() - poseest.pos()).norm();
                printf("ID %d DXYZ %4.2f %4.2f %4.2f ESTDXYZ %4.2f %4.2f %4.2f",_idj, det.pos().x(), det.pos().y(), det.pos().z(), DposeEST.pos().x(), DposeEST.pos().y(), DposeEST.pos().z());
            }

        }

        printf("\n");
    }    
}

void SwarmLocalizationSolver::add_as_keyframe(const SwarmFrame &sf) {
    // if (sf_sld_win.size() > 0) {
        // last_kf_ts = sf_sld_win.back().ts;
    // }

    sf_sld_win.push_back(sf);
    all_sf[sf.ts] = sf;
    ROS_INFO("\n\nNew keyframe %d; Details", TSShort(sf.ts));
    for (auto it : sf.id2nodeframe) {
        if (it.second.is_static) {
            ROS_INFO("Is static");
            this->init_static_nf_in_keyframe(sf.ts, it.second);
        } else {
            auto _nf = it.second;
            double _pose[4];
            _nf.pose().to_vector_xyzyaw(_pose);
            printf("ID %d pose %f %f %f %f; DIS:", _nf.id, _pose[0], _pose[1], _pose[2], _pose[3]);
            for (auto it2: _nf.dis_map) {
                printf("%d %f;", it2.first, it2.second);
            }
            printf("\n");
            this->init_dynamic_nf_in_keyframe(sf.ts, it.second);
        }
    }

    print_frame(sf);

    last_kf_ts = sf.ts;
    has_new_keyframe = true;
}

void SwarmLocalizationSolver::replace_last_kf(const SwarmFrame &sf) {
    delete_frame_i(sf_sld_win.size()-1);
    sf_sld_win.push_back(sf);
    all_sf[sf.ts] = sf;

    for (auto it : sf.id2nodeframe) {
        if (it.second.is_static) {
            ROS_INFO("Is static");
            this->init_static_nf_in_keyframe(sf.ts, it.second);
        } else {
            this->init_dynamic_nf_in_keyframe(sf.ts, it.second);
        }
    }
    last_kf_ts = sf.ts;
    has_new_keyframe = true;
}


void SwarmLocalizationSolver::add_new_swarm_frame(const SwarmFrame &sf) {
    process_frame_clear();
    if (detect_outlier(sf)) {
        ROS_INFO("Outlier detected!");
        return;
    }

    auto _ids = sf.node_id_list;
    for (int _id : _ids) {
        all_nodes.insert(_id);
    }

    int is_kf = judge_is_key_frame(sf);
    if (is_kf == 1) {
        add_as_keyframe(sf);
        ROS_INFO("New kf found, sld win size %ld TS %d NFTS %d ID: [", sf_sld_win.size(),
            TSShort(sf_sld_win.back().ts),
            TSShort(sf_sld_win.back().id2nodeframe[self_id].ts)
        );
        for (int _id : _ids) {
            printf(" %d", _id);
        }
        printf("]\n");
    }

#ifdef ENABLE_REPLACE
    if (is_kf == 2) {
        replace_last_kf(sf);
        ROS_INFO("Replace last kf with TS %d",  TSShort(sf_sld_win.back().ts));
    }
#endif

    if (_ids.size() > drone_num) {
        //For here the drone num increase
        drone_num = _ids.size();
    }

    
}


std::pair<Pose, Eigen::Matrix4d> SwarmLocalizationSolver::PredictNode(const NodeFrame & nf, bool attitude_yaw_only) const {
    std::pair<Pose, Eigen::Matrix4d> ret; 
    if (last_saved_est_kf_ts > 0 && finish_init &&
        est_poses_tsid_saved.at(last_saved_est_kf_ts).find(nf.id)!=est_poses_tsid_saved.at(last_saved_est_kf_ts).end() ) {

        //Use last solve relative res, e.g init with last
        int _id = nf.id;
        Pose _last = Pose(est_poses_tsid_saved.at(last_saved_est_kf_ts).at(_id), true);

        Pose last_vo = all_sf.at(last_saved_est_kf_ts).id2nodeframe.at(_id).pose();
        Pose now_vo = nf.pose();

        if (attitude_yaw_only) {
            now_vo.set_yaw_only();
            last_vo.set_yaw_only();
        }
        Eigen::Isometry3d TnowVO = now_vo.to_isometry();
        Eigen::Isometry3d TlastVO = last_vo.to_isometry();


        Eigen::Isometry3d Tlast = _last.to_isometry();

        Pose transfered_now(Tlast*TlastVO.inverse()*TnowVO);

        ret.first = transfered_now;
        // ROS_INFO("Request cov %ld %d", last_saved_est_kf_ts, nf.id);
#ifdef COMPUTE_COV
        if (nf.id != self_id){
            ret.second = est_cov_tsid.at(last_saved_est_kf_ts).at(nf.id);
        } else {
            ret.second = Eigen::Matrix4d::Zero();
        }
#else
    ret.second = Eigen::Matrix4d::Zero();
#endif

    } else {
        ROS_ERROR("Can't Predict Node Pose: Not inited");
        exit(-1);
        return ret;
    }
    return ret;
}


std::pair<Pose, Eigen::Matrix4d> SwarmLocalizationSolver::NodeCooridnateOffset(int _id, bool attitude_yaw_only) const {
    std::pair<Pose, Eigen::Matrix4d> ret; 
    if (last_saved_est_kf_ts > 0 && finish_init &&
        est_poses_tsid_saved.at(last_saved_est_kf_ts).find(_id)!=est_poses_tsid_saved.at(last_saved_est_kf_ts).end() ) {

        Pose PBA = Pose(est_poses_tsid_saved.at(last_saved_est_kf_ts).at(_id), true);

        Pose PBB = all_sf.at(last_saved_est_kf_ts).id2nodeframe.at(_id).pose();
        
        PBA.set_yaw_only();
        PBB.set_yaw_only();
        
        ret.first = Pose(PBA.to_isometry() * PBB.to_isometry().inverse());
        // ROS_INFO("Request cov %ld %d", last_saved_est_kf_ts, nf.id);
#ifdef COMPUTE_COV
        if (_id != self_id){
            auto cov = est_cov_tsid.at(last_saved_est_kf_ts).at(_id);
            cov.block<3,3>(0,0) = PBA.to_isometry().rotation()*cov.block<3,3>(0,0);
            ret.second = cov;
        } else {
            ret.second = Eigen::Matrix4d::Zero();
        }
#else
    ret.second = Eigen::Matrix4d::Zero();
#endif

    } else {
        ROS_ERROR("Can't Predict Node Pose: Not inited");
        exit(-1);
        return ret;
    }
    return ret;
}


SwarmFrameState SwarmLocalizationSolver::PredictSwarm(const SwarmFrame &sf) const {
    SwarmFrameState sfs;
    if(!finish_init) {
        ROS_WARN("Predict swarm poses failed: SwarmLocalizationSolver not inited\n");
        return sfs;
    }
    
    for (auto it : sf.id2nodeframe) {
        int _id = it.first;

        NodeFrame & nf = it.second;
        if (est_poses_tsid_saved.find(last_saved_est_kf_ts) != est_poses_tsid_saved.end()) {
            if(est_poses_tsid_saved.at(last_saved_est_kf_ts).find(nf.id)!=est_poses_tsid_saved.at(last_saved_est_kf_ts).end()) {
                auto ret = this->PredictNode(nf);
                sfs.node_poses[_id] = ret.first;
                sfs.node_covs[_id] = ret.second;
                //Give node velocity predict here
                sfs.node_vels[_id] = Eigen::Vector3d(0, 0, 0);
                auto ret2 = this->NodeCooridnateOffset(nf.id);
                sfs.base_coor_poses[_id] = ret2.first;
                sfs.base_coor_covs[_id] = ret2.second;
            } else {
                // Maybe use previous results
                // ROS_WARN("No id %d found in last kf", nf.id);
            }
        }
    }

    return sfs;
}


bool SwarmLocalizationSolver::solve_with_multiple_init(int max_number) {

    double cost = acpt_cost;
    bool cost_updated = false;

    ROS_WARN("Try to use %d random init to solve expect cost %f", max_number, cost);
    //Need to rewrite here to enable multiple trial of input!!!
    EstimatePoses _est_poses_best;// = est_poses_tsid;
    EstimatePoses & _est_poses = est_poses_tsid;
    EstimatePosesIDTS & _est_poses_idts = est_poses_idts;
 
    for (int i = 0; i < max_number; i++) {
        ROS_WARN("%d time of init trial", i);
        random_init_pose(_est_poses,  _est_poses_idts);
        double c = solve_once(_est_poses,  _est_poses_idts,  true);

        if (c < cost) {
            ROS_INFO("Got better cost %f", c);
            cost_updated = true;
            cost_now = cost = c;
            // return true;
            for (auto it : est_poses_tsid) {
                auto ts = it.first;
                if (_est_poses_best.find(ts) != _est_poses_best.end()) {
                    for (auto it2: _est_poses_best[ts]) {
                        delete [] (_est_poses_best[ts][it2.first]);
                    }
                }
                _est_poses_best[ts] = std::map<int, double*>();
                for (auto it2: it.second) {
                    auto _id = it2.first;
                    double * _p = new double[4];
                    _est_poses_best[ts][_id] = _p;
                    memcpy(_p, _est_poses[ts][_id], 4*sizeof(double));
                }
            }
        }
    }

    if (cost_updated) {
        for (auto it : est_poses_tsid) {
            auto ts = it.first;
            for (auto it2: it.second) {
                auto _id = it2.first;
                memcpy(_est_poses[ts][_id], _est_poses_best[ts][_id], 4*sizeof(double));
                delete [] (_est_poses_best[ts][_id]);
            }
        }
    }

    return cost_updated;
}


std::pair<Eigen::Vector3d, Eigen::Vector3d> SwarmLocalizationSolver::boundingbox_sldwin(int _id) const {
    double xmax=-1000, xmin = 1000, ymax = -1000, ymin = 1000, zmax = -1000, zmin = 1000;
    for (const SwarmFrame & _sf : sf_sld_win ) {
        if (_sf.HasID(_id) && _sf.id2nodeframe.at(_id).vo_available ) {
            Vector3d pos = _sf.id2nodeframe.at(_id).position();
            if (pos.x() > xmax) {
                xmax = pos.x();
            }
            if (pos.y() > ymax) {
                ymax = pos.y();
            }
            if (pos.z() > zmax) {
                zmax = pos.z();
            }

            if (pos.x() < xmin) {
                xmin = pos.x();
            }
            if (pos.y() < ymin) {
                ymin = pos.y();
            }
            if (pos.z() < zmin) {
                zmin = pos.z();
            }
        }
    }

    return std::make_pair(Eigen::Vector3d(xmin, ymin, zmin),
        Eigen::Vector3d(xmax, ymax, zmax));
}

double SwarmLocalizationSolver::solve() {
    if (self_id < 0 || sf_sld_win.size() < min_frame_number)
        return -1;

    if (!has_new_keyframe)
        return cost_now;

    // if (!finish_init || drone_num > last_drone_num) {
    // Should not reinit when drone_num > last drone num
    if (!finish_init) {
        //Init procedure
        auto bbx = boundingbox_sldwin(self_id);
        auto min = bbx.first;
        auto max = bbx.second;
        if (max.x() - min.x() > init_xy_movement &&
            max.y() - min.y() > init_xy_movement &&
            max.z() - min.z() > init_z_movement
        ) {
            ROS_INFO("No init before, try to init");
            finish_init = solve_with_multiple_init(INIT_TRIAL);
            if (finish_init) {
                last_drone_num = drone_num;
                ROS_INFO("Finish init\n");
            }
        } else {
            ROS_WARN("BOUNDING BOX too small; Pending more movement");
            return -1;
        }
       
    } else if (has_new_keyframe) {
        ROS_INFO("New keyframe, solving....");
        cost_now = solve_once(this->est_poses_tsid, this->est_poses_idts, true);
    }

    if (cost_now > acpt_cost) {
        finish_init = false;
    }

    if (finish_init) {
        sync_est_poses(this->est_poses_tsid);
    }

    return cost_now;
}

void  SwarmLocalizationSolver::sync_est_poses(const EstimatePoses &_est_poses_tsid) {
    ROS_INFO("Start sync poses to saved while init successful");
    int64_t last_ts = sf_sld_win.back().ts;
    for (const SwarmFrame & sf : sf_sld_win) {
        //Only update param in sf to saved
        for (auto it : sf.id2nodeframe) {
            int _id = it.first;
            const NodeFrame _nf = it.second;
            if (est_poses_tsid_saved.find(sf.ts) == est_poses_tsid_saved.end()) {
                est_poses_tsid_saved[sf.ts] = std::map<int,double*>();
            }
            if (est_poses_idts_saved.find(_id) == est_poses_idts_saved.end()) {
                est_poses_idts_saved[_id] = std::map<int64_t,double*>();
            }

            if (est_poses_tsid_saved[sf.ts].find(_id) == est_poses_tsid_saved[sf.ts].end()) {
                est_poses_tsid_saved[sf.ts][_id] = new double[4];
            }

            if (est_poses_idts_saved[_id].find(sf.ts) == est_poses_idts_saved[_id].end()) {
                est_poses_idts_saved[_id][sf.ts] = new double[4];
            }  
            
            if (_est_poses_tsid.find(sf.ts) !=_est_poses_tsid.end() &&
                _est_poses_tsid.at(sf.ts).find(_id) != _est_poses_tsid.at(sf.ts).end()
            ) {
                last_ts = sf.ts;
                memcpy(est_poses_tsid_saved[sf.ts][_id], _est_poses_tsid.at(sf.ts).at(_id), 4*sizeof(double));
                memcpy(est_poses_idts_saved[_id][sf.ts], _est_poses_tsid.at(sf.ts).at(_id), 4*sizeof(double));
            } 
        }
    }
    ROS_INFO("finish sync");

    last_saved_est_kf_ts = last_ts;
}

unsigned int SwarmLocalizationSolver::sliding_window_size() const {
    return sf_sld_win.size();
}

CostFunction *
SwarmLocalizationSolver::_setup_cost_function_by_sf(const SwarmFrame &sf, std::map<int, int> id2poseindex, bool is_lastest_frame, int & res_num) const {
    //Here we will only send
    std::map<int, double> yaw_init;
    for (const auto & it : sf.id2nodeframe) {
//            auto _id = it.first;
        const NodeFrame &_nf = it.second;
        //First we come to distance error
        if (_nf.frame_available) {
            yaw_init[_nf.id] = est_poses_tsid.at(_nf.ts).at(_nf.id)[3];
        }
    }
    SwarmFrameError * sferror = new SwarmFrameError(sf, id2poseindex, yaw_observability, yaw_init, is_lastest_frame, !finish_init);
    res_num = sferror->residual_count();
    auto cost_function  = new SFErrorCost(sferror);
    
    for (auto it : id2poseindex) {
        int _id = it.first;
        if (!finish_init) {
            //When not finish init; only estimate XY position
            cost_function->AddParameterBlock(2);
        } else {
            if (!yaw_observability.at(_id)) {
                cost_function->AddParameterBlock(3);
            } else {
                cost_function->AddParameterBlock(4);
            }
        }
    }

    assert(res_num > 0 &&"Set cost function with SF has 0 res num");
    cost_function->SetNumResiduals(res_num);
    return cost_function;
}



void SwarmLocalizationSolver::setup_problem_with_sferror(const EstimatePoses & swarm_est_poses, Problem& problem, const SwarmFrame& sf, TSIDArray& param_indexs, bool is_lastest_frame) const {
    //TODO: Deal with static object in this function!!!
    std::vector<double*> pose_state;
    std::map<int, int> id2poseindex;
    int64_t ts = sf.ts;
    for(auto it : sf.id2nodeframe) {
        int _id = it.first;
        if (is_lastest_frame && _id == self_id) {
            continue;
        } else {
            // ROS_INFO("Add TS %d ID %d", TSShort(ts), _id);
            pose_state.push_back(swarm_est_poses.at(ts).at(_id));
            id2poseindex[_id] = pose_state.size() - 1;
            param_indexs.push_back(std::pair<int64_t, int>(ts, _id));
        }
    }
    int res_num = 0;
    CostFunction * cost = _setup_cost_function_by_sf(sf, id2poseindex, is_lastest_frame, res_num);
    problem.AddResidualBlock(cost, nullptr, pose_state);

    if (finish_init) {
        printf("SF Evaluate ERROR ts %d", TSShort(ts));
        double * res = new double[res_num];
        cost->Evaluate(pose_state.data(), res, nullptr);
        for (int i = 0; i < res_num; i++) {
            printf(" %f ", res[i]);
        }
        printf("\n");
    }
}

CostFunction *
SwarmLocalizationSolver::_setup_cost_function_by_nf_win(std::vector<NodeFrame> &nf_win, const std::map<int64_t, int> & ts2poseindex, bool is_self) const {
    int _id = nf_win[0].id;
    std::vector<double> yaw_init;
    for (NodeFrame & _nf : nf_win) {
        yaw_init.push_back(est_poses_tsid.at(_nf.ts).at(_nf.id)[3]);
    }
    auto she = new SwarmHorizonError(nf_win, ts2poseindex, yaw_observability.at(_id), yaw_init, is_self, !finish_init);
    auto cost_function = new HorizonCost(she);
    int res_num = she->residual_count();

    int poses_num = nf_win.size();
    if (is_self) {
        poses_num = nf_win.size() - 1;
    }

    for (int i =0;i < poses_num; i ++) {
        if (!finish_init) {
            cost_function->AddParameterBlock(2);
        } else {
            if (!yaw_observability.at(_id)) {
                cost_function->AddParameterBlock(3);
            } else {
                cost_function->AddParameterBlock(4);
            }
        }
    }
//        ROS_INFO("SFHorizon res %d", res_num);
    if (res_num == 0) {
        ROS_WARN("Set cost function with NF has 0 res num; NF id %d WIN %ld", nf_win[0].id, nf_win.size());
        // exit(-1);
        return nullptr;
    }

    cost_function->SetNumResiduals(res_num);
    return cost_function;
}

void SwarmLocalizationSolver::setup_problem_with_sfherror(const EstimatePosesIDTS & est_poses_idts, Problem& problem, int _id) const {
    auto nfs = est_poses_idts.at(_id);

    if (nfs.size() < 2) {
        ROS_INFO("Frame nums for id %d is to small:%ld", _id, nfs.size());
        return;
    }

    std::vector<NodeFrame> nf_win;
    std::vector<double*> pose_win;
    std::map<int64_t, int> ts2poseindex;

    // for (auto it: nfs) {
    for (const SwarmFrame & sf : sf_sld_win) {
        int64_t ts = sf.ts;
        if (nfs.find(ts) != nfs.end()) {
            auto _p = nfs[ts];
            if (pose_win.size() < 1 || pose_win[pose_win.size()-1] != _p) {
                pose_win.push_back(nfs[ts]);
                const NodeFrame & _nf = all_sf.at(ts).id2nodeframe.at(_id);
                if(_nf.is_static) {
                    return;
                }
                nf_win.push_back(_nf);
                ts2poseindex[ts] = nf_win.size() - 1;
            }

            // ROS_INFO("Add TS %d ID %d", TSShort(ts), _id);
            
        } 

    }

    if (_id == self_id) {
        //Delete last one that don't need to estimate
        pose_win.erase(pose_win.end() - 1);

        //Not estimate myself; useless
        for (double * _p : pose_win) {
            problem.SetParameterBlockConstant(_p);
        }
        return;
    }

    CostFunction * cf = _setup_cost_function_by_nf_win(nf_win, ts2poseindex, _id==self_id);
    if (cf != nullptr) {
        problem.AddResidualBlock(cf , nullptr, pose_win);
    }
}

bool SwarmLocalizationSolver::NFnotMoving(const NodeFrame & _nf1, const NodeFrame & _nf2) const {
    Eigen::Vector3d _diff = _nf1.position() - _nf2.position();
    //TODO: make it set to if last dont's have some detection and this frame has, than keyframe
    if (_diff.norm() > NOT_MOVING_THRES) {
        return false;
    }
    return true;
}

void SwarmLocalizationSolver::cutting_edges() {
    //TODO: deal with fist sf
    int detection_count = 0;
    int total_detection_count = 0;

    int distance_count = 0;
    int total_distance_count = 0;

    SwarmFrame & sf0 = sf_sld_win[0];
    for (auto & it : sf0.id2nodeframe) {
        auto & _nf = it.second;
        _nf.enabled_distance.clear();
        for (auto it_dis : _nf.dis_map) {
            int _id2 = it_dis.first;
            _nf.enabled_distance[_id2] = true;
            distance_count += 1;
            total_distance_count += 1;
        }

        for (auto it_de : _nf.detected_nodes) {
            int _id2 = it_de.first;
            _nf.enabled_detection[_id2] = true;
            detection_count += 1;
            total_detection_count += 1;
         }
        //ROS_WARN("TS %d ID %d ENABLED %ld DISMAP %ld\n", TSShort(_nf.ts), _nf.id, _nf.dis_map.size(), _nf.enabled_distance.size());
    }

    for (unsigned int i = 1; i < sf_sld_win.size(); i++) {
        SwarmFrame & sf = sf_sld_win[i];
        SwarmFrame & last_sf = sf_sld_win[i - 1];
        std::set<int> moved_nodes; //Mark the node not moved from last sf

        for (auto & it : sf.id2nodeframe) {
            auto _nf = it.second;
            auto _id = it.first;
            if (!last_sf.HasID(_id) || 
                !NFnotMoving(last_sf.id2nodeframe[_id], _nf)
                ) {
                moved_nodes.insert(_id);
            }
        }
        // Now we have all moved node; Let's begin with edging enabling
        for (auto & it : sf.id2nodeframe) {
            NodeFrame & _nf = it.second;
            auto _id = it.first;
            _nf.enabled_distance.clear();
            for (auto it_dis : _nf.dis_map) {
                int _id2 = it_dis.first;
                _nf.enabled_distance[_id2] = false;
                total_distance_count += 1;
                if ((moved_nodes.find(_id) != moved_nodes.end() ||
                    moved_nodes.find(_id2) != moved_nodes.end())) {                    
                    if( sf.HasID(_id2) && 
                        (sf.id2nodeframe[_id2].enabled_distance.find(_id) == sf.id2nodeframe[_id2].enabled_distance.end() || !sf.id2nodeframe[_id2].enabled_distance[_id])) {
                        //ROS_INFO("Merging distanc %3.2f and %3.2f to %3.2f", 
                        //    _nf.dis_map[_id2],
                        //    sf.id2nodeframe[_id2].dis_map[_id],
                        //    (_nf.dis_map[_id2] + sf.id2nodeframe[_id2].dis_map[_id])/2.0
                        //);
                        _nf.dis_map[_id2] = (_nf.dis_map[_id2] + sf.id2nodeframe[_id2].dis_map[_id])/2.0;
                        _nf.enabled_distance[_id2] = true;
                        distance_count += 1;
                    }
                }
            }

            for (auto it_de : _nf.detected_nodes) {
                int _id2 = it_de.first;
                _nf.enabled_detection[_id2] = true;
                total_detection_count += 1;
                if (last_sf.HasDetect(_id, _id2) && (moved_nodes.find(_id) == moved_nodes.end() && moved_nodes.find(_id2) == moved_nodes.end())
                ) {
                    _nf.enabled_detection[_id2] = false;
                    //ROS_INFO("TS %d DET FROM %d TO %d throw", TSShort(sf.ts), _id, _id2);
                } else {
                    detection_count += 1;
                    //ROS_INFO("TS %d DET FROM %d TO %d USE", TSShort(sf.ts), _id, _id2);
                }
            }
        }
    }

    ROS_INFO("Edge Optimized DIS %d(%d) DET %d(%d)", distance_count, total_distance_count, detection_count, total_detection_count);
    /*
    for (auto & sf : sf_sld_win) {
        for (auto & it : sf.id2nodeframe) {
            auto _nf = it.second;
            auto _id = it.first;
            ROS_WARN("TS %d ID %d ENABLED %ld DISMAP %ld\n", TSShort(_nf.ts), _nf.id, _nf.dis_map.size(), _nf.enabled_distance.size());
        }
    }*/
}

void SwarmLocalizationSolver::estimate_yaw_observability() {
    yaw_observability.clear();
    if (!finish_init) {
        printf("YAW observability: ");
        for (int _id : all_nodes) {
            yaw_observability[_id] = false;
            printf("%d: %s ", _id, yaw_observability[_id]?"true":"false");
        }
        printf("\n");
        return;
    }
    printf("YAW observability: ");
    for (int _id: all_nodes) {
        auto bbx = boundingbox_sldwin(_id);
        auto min = bbx.first;
        auto max = bbx.second;

        if (max.x() - min.x() > THRES_YAW_OBSER_XY || max.y() - min.y() > THRES_YAW_OBSER_XY) {
            yaw_observability[_id] = true;
        } else {
            yaw_observability[_id] = false;
        }

        printf("%d: %s ", _id, yaw_observability[_id]?"true":"false");
    }

    printf("\n");
}


double SwarmLocalizationSolver::solve_once(EstimatePoses & swarm_est_poses, EstimatePosesIDTS & est_poses_idts, bool report) {

    ros::Time t1 = ros::Time::now();
    Problem problem;

//        if (solve_count % 10 == 0)
    printf("SOLVE COUNT %d Trying to solve size %d, TS %ld\n", solve_count, sliding_window_size(), swarm_est_poses.size());
    estimate_yaw_observability();
    has_new_keyframe = false;
    std::vector<std::pair<int64_t, int>> param_indexs;
    cutting_edges();


    for (unsigned int i = 0; i < sf_sld_win.size(); i++ ) {
        // ROS_INFO()
        this->setup_problem_with_sferror(swarm_est_poses, problem, sf_sld_win[i], param_indexs, i==sf_sld_win.size()-1);
    }

    int num_res_blks_sf = problem.NumResidualBlocks();
    int num_res_sf = problem.NumResiduals();
    ROS_INFO("SF residual blocks %d residual nums %d", num_res_blks_sf, num_res_sf);

    for (int _id: all_nodes) {
        this->setup_problem_with_sfherror(est_poses_idts, problem, _id);       
    }

    ROS_INFO("SFH residual blocks %d residual nums %d", problem.NumResidualBlocks() - num_res_blks_sf, problem.NumResiduals() - num_res_sf);


    ceres::Solver::Options options;

    //SPARSE NORMAL DOGLEG 12.5ms
    //SPARSE NORMAL 21
    //DENSE NORM DOGLEG 49.31ms
    options.max_num_iterations = 1000;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    if (finish_init) {
        options.max_solver_time_in_seconds = MAX_SOLVER_TIME;
    }else {
    }
    options.num_threads = thread_num;
    Solver::Summary summary;

    options.trust_region_strategy_type = ceres::DOGLEG;
    
    ros::Time t2 = ros::Time::now();

    ceres::Solve(options, &problem, &summary);

    double equv_cost = summary.final_cost / sliding_window_size();

    equv_cost = equv_cost / (double) (drone_num * (drone_num - 1));
    equv_cost = sqrt(equv_cost)/ERROR_NORMLIZED;
    if (!report) {
        return equv_cost;
    }

    std::cout << "\nSize:" << sliding_window_size() << "\n" << summary.BriefReport() << " Equv cost : "
              << equv_cost << " Time : " << summary.total_time_in_seconds * 1000 << "ms\n\n\n";
    //std::cout << summary.FullReport() << std::endl;

#ifdef DEBUG_OUTPUT_POSES
    if (finish_init) {

        for (auto it : est_poses_idts) {
            auto id = it.first;
            ROS_INFO("\n\nID %d ", id);
            double* pose_last = nullptr;
            Pose pose_vo_last;
            // for(auto it2 : it.second) {
                // auto ts = it2.first;
                // double * pose = it2.second;
                auto ts = sf_sld_win.back().ts;
                if (est_poses_tsid[ts].find(id) == est_poses_tsid[ts].end()) {
                    continue;
                }

                double * pose = est_poses_tsid[ts][id];
                auto pose_vo = all_sf[ts].id2nodeframe[id].pose();
                auto poseest = Pose(pose, true);
                ROS_INFO("TS %d POS %3.4f %3.4f %3.4f YAW %5.4fdeg", TSShort(ts), pose[0], pose[1], pose[2], wrap_angle(pose[3])*57.3);
                ROS_INFO("POSVO        %3.4f %3.4f %3.4f YAW %5.4fdeg",
                        pose_vo.pos().x(), pose_vo.pos().y(), pose_vo.pos().z(), pose_vo.yaw()*57.3);
                ROS_INFO("POSVOEST     %3.4f %3.4f %3.4f YAW %5.4fdeg",
                        poseest.pos().x(), poseest.pos().y(), poseest.pos().z(), pose_vo.yaw()*57.3);
                if (pose_last!=nullptr) {
                    Pose DposeVO = Pose::DeltaPose(pose_vo_last, pose_vo, true);
                    Pose DposeEST = Pose::DeltaPose(Pose(pose_last, true), Pose(pose, true), true);
                    Pose ERRVOEST = Pose::DeltaPose(DposeVO, DposeEST, true);
                    double ang_err = ERRVOEST.yaw()/ VO_ERROR_ANGLE;
                    
                    ROS_WARN("ERRVOEST       %6.5f %6.5f %6.5f ANG  %3.2f",
                            ERRVOEST.pos().x()/VO_DRIFT_METER, ERRVOEST.pos().y()/VO_DRIFT_METER, ERRVOEST.pos().z()/VO_DRIFT_METER, ang_err);

                    ROS_INFO("DPOSVO         %6.5f %6.5f %3.4f YAW %5.4fdeg",
                            DposeVO.pos().x(), DposeVO.pos().y(), DposeVO.pos().z(), DposeVO.yaw()*57.3);

                    ROS_INFO("DPOSEST        %6.5f %6.5f %3.4f YAW %5.4fdeg\n\n",
                            DposeEST.pos().x(), DposeEST.pos().y(), DposeEST.pos().z(), DposeEST.yaw()*57.3);
                }

                printf("DISTANCES ");
                for (auto itj : all_sf[ts].id2nodeframe[id].dis_map) {
                    int _idj = itj.first;
                    double dis = itj.second;
                    if (all_sf[ts].id2nodeframe[_idj].vo_available) {
                        Pose posj_vo = all_sf[ts].id2nodeframe[_idj].pose();
                        Pose posj_est(est_poses_idts[_idj][ts], true);
                        double vo_dis = (posj_vo.pos() - pose_vo.pos()).norm();
                        double est_dis = (posj_est.pos() - Pose(pose, true).pos()).norm();
                        printf(" DIS %4.2f VO %4.2f EST %4.2f ", dis, vo_dis, est_dis);
                    }

                }

                printf("\n");


                pose_last = pose;
                pose_vo_last = pose_vo;
            // }
        }

    }
#endif

    solve_time_count += summary.total_time_in_seconds;
    solve_count++;

    ROS_INFO("Average solve time %3.2fms", solve_time_count *1000 / solve_count);
    // exit(-1);
    ros::Time t3 = ros::Time::now();
#ifdef COMPUTE_COV
    this->compute_covariance(problem, param_indexs);    
#endif
    //Use this jacobian to give a covariance function for each state. than add marginalization
    ros::Time t4 = ros::Time::now();

    ROS_INFO("Dt1 %3.2f ms DT2 %3.2f DT3 %3.2f TOTAL %3.2f",
        (t2-t1).toSec()*1000,
        (t3-t2).toSec()*1000,
        (t4-t3).toSec()*1000,
        (ros::Time::now() - t1).toSec()*1000
    );

    return equv_cost;
}

Eigen::MatrixXd CRSMatrixToEigenMatrix(const ceres::CRSMatrix &crs_matrix);

void SwarmLocalizationSolver::compute_covariance(Problem & problem, TSIDArray param_indexs) {

    //This function still has bug when static

    ceres::Covariance::Options cov_options;
    // cov_options.algorithm_type = DENSE_SVD;
    // cov_options.null_space_rank = 1;
    Covariance covariance(cov_options);
    std::vector<std::pair<const double*, const double*> > covariance_blocks;

    Problem::EvaluateOptions evaluate_options_;
    evaluate_options_.num_threads = 1;
    evaluate_options_.apply_loss_function = true;
    CRSMatrix jacobian;
    problem.Evaluate(evaluate_options_, NULL, NULL, NULL, &jacobian);
    auto jacb = CRSMatrixToEigenMatrix(jacobian);
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    // ROS_INFO("ID %d Mat rows %ld cols %ld\n", self_id, jacb.rows(), jacb.cols());
    auto JtJ = (jacb.transpose()*jacb).sparseView();
    Eigen::SimplicialLLT <Eigen::SparseMatrix<double>> solver;
    
    solver.compute(JtJ);
    Eigen::SparseMatrix<double> I(JtJ.rows(), JtJ.rows());
    I.setIdentity();
    Eigen::MatrixXd cov = Eigen::MatrixXd(solver.solve(I))*ERROR_NORMLIZED*ERROR_NORMLIZED;
    // ROS_INFO("COV size %d %d", cov.cols(), cov.rows());
    // std::cout << cov << std::endl;
    for (int i = 0; i < 4; i++) {
        for (int j =0; j < 4; j++) {
            if (cov(i,j) > 10) {
                cov(i, j) = 10;
            }
            if (cov(i,j) < 0) {
                cov(i,j) = 0;
            }
        }
    }
    
    //Init self this last with 0
    if (est_cov_tsid.find(sf_sld_win.back().ts) == est_cov_tsid.end()) {
        est_cov_tsid[sf_sld_win.back().ts] = std::map<int,Eigen::Matrix4d>();
    }

    est_cov_tsid[sf_sld_win.back().ts][self_id] = Eigen::Matrix4d::Zero();

    for (int j = 0; j < cov.cols() / 4; j ++ ) {
        int64_t _ts = param_indexs[j].first;
        int _id = param_indexs[j].second;
        if (est_cov_tsid.find(_ts) == est_cov_tsid.end()) {
            est_cov_tsid[_ts] = std::map<int,Eigen::Matrix4d>();
        }
        // ROS_INFO("set cov %ld %d", _ts, _id);

        est_cov_tsid[_ts][_id] = cov.block<4, 4>(4*j,4*j);
        // int j = 0;
        // if (param_indexs[j].first == ts ) {
#ifdef DEBUG_OUTPUT_COV
        printf("\nTS %d ", TSShort(_ts));
        printf("ID %d SD %4.3lf %4.3lf %4.3lf %4.3lf", _id, cov(4*j,4*j), 
                cov(4*j+1,4*j+1), 
                cov(4*j+2,4*j+2), 
                cov(4*j+3,4*j+3));
            // fflush(stdout);
        // }
#endif
    }

    printf("\n");
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    ROS_INFO("Eigen compute covariance %4.3fms\n", duration/1000.0);

}

Eigen::MatrixXd CRSMatrixToEigenMatrix(const ceres::CRSMatrix &crs_matrix) {
    Eigen::MatrixXd eigen_matrix;
    eigen_matrix.resize(crs_matrix.num_rows, crs_matrix.num_cols);
    eigen_matrix.setZero();
    for (int row = 0; row < crs_matrix.num_rows; ++row) {
        int start = crs_matrix.rows[row];
        int end = crs_matrix.rows[row + 1] - 1;
        
        for (int i = start; i <= end; i++) {
            int col = crs_matrix.cols[i];
            double value = crs_matrix.values[i];
            (eigen_matrix)(row, col) = value;
        }
    }
    return eigen_matrix;
}