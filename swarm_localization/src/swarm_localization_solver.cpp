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

// #define DEBUG_OUTPUT_POSES
#define COMPUTE_COV
#define SMALL_MOVEMENT_SPD 0.1
#define REPLACE_MIN_DURATION 0.1
#define ENABLE_REPLACE
#define MAX_SOLVER_TIME 0.1
#define ENABLE_HISTORY_COV


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


    if (sf.HasID(self_id) && last_sf.HasID(self_id) && sf.has_odometry(self_id) && last_sf.has_odometry(self_id)) {

        Eigen::Vector3d _diff = sf.position(self_id) - last_sf.position(self_id);

        //TODO: make it set to if last dont's have some detection and this frame has, than keyframe
        if (_diff.norm() > min_accept_keyframe_movement || 
            (_diff.norm() > min_accept_keyframe_movement*0.5 && sf.has_detect() > 0)   ){ //(sf.HasDetect(_id) && _id==self_id))
            ret.push_back(self_id);
            node_kf_count[self_id] += 1;
            ROS_INFO("SF %d is kf of %d: DIFF %3.2f HAS %d", 
                TSShort(sf.ts), self_id, _diff.norm(), sf.HasDetect(self_id));

            return 1;
        }

        if (sf.swarm_size() >= last_sf.swarm_size() && _diff.norm() < SMALL_MOVEMENT_SPD * dt && sf.swarm_size() > last_sf.swarm_size() &&
        ( (dt > REPLACE_MIN_DURATION && sf.has_detect() == last_sf.has_detect()) || sf.has_detect() > last_sf.has_detect())
        ) {
            //Make sure is fixed
            return 2;
        }

    } else {
        ROS_ERROR("No self id :%d last %d this %d ODOM %d %d", self_id, sf.HasID(self_id), last_sf.HasID(self_id), sf.has_odometry(self_id), last_sf.has_odometry(self_id));
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
            double * p = it2.second;
            p[0] = rand_FloatRange(-10, 10);
            p[1] = rand_FloatRange(-10, 10);
            p[2] = rand_FloatRange(0, 0);
            p[3] = rand_FloatRange(-M_PI, M_PI);
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

            Pose transfered_now(Tlast*TlastVO.inverse()*TnowVO);


            transfered_now.to_vector_xyzyaw(_p);

        } else {

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

void SwarmLocalizationSolver::add_as_keyframe(const SwarmFrame &sf) {
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

SwarmFrameState SwarmLocalizationSolver::PredictSwarm(const SwarmFrame &sf) const {
    SwarmFrameState sfs;
    if(!finish_init) {
        ROS_WARN("Predict swarm poses failed: SwarmLocalizationSolver not inited\n");
        return sfs;
    }
    
    for (auto it : sf.id2nodeframe) {
        int _id = it.first;

        NodeFrame & nf = it.second;
        if(est_poses_tsid_saved.at(last_saved_est_kf_ts).find(nf.id)!=est_poses_tsid_saved.at(last_saved_est_kf_ts).end()) {
            auto ret = this->PredictNode(nf);
            sfs.node_poses[_id] = ret.first;
            sfs.node_covs[_id] = ret.second;
            //Give node velocity predict here
            sfs.node_vels[_id] = Eigen::Vector3d(0, 0, 0);
        } else {
            // Maybe use previous results
            // ROS_WARN("No id %d found in last kf", nf.id);
        }
    }

    return sfs;
}


bool SwarmLocalizationSolver::solve_with_multiple_init(int start_drone_num, int min_number, int max_number) {

    double cost = acpt_cost;
    bool cost_updated = false;

    ROS_WARN("Try to use multiple init to solve expect cost %f", cost);

    EstimatePoses & _est_poses = est_poses_tsid;
    EstimatePosesIDTS & _est_poses_idts = est_poses_idts;

    for (int i = 0; i < max_number; i++) {
        random_init_pose(_est_poses,  _est_poses_idts);
        double c = solve_once(_est_poses,  _est_poses_idts,  true);
        ROS_INFO("Got better cost %f", c);

        if (c < cost) {
            ROS_INFO("Got better cost %f", c);
            cost_updated = true;
            cost_now = cost = c;
            return true;
        }
    }

    return cost_updated;
}

double SwarmLocalizationSolver::solve() {
    if (self_id < 0 || sf_sld_win.size() < min_frame_number)
        return -1;

    if (!has_new_keyframe)
        return cost_now;

    if (!finish_init || drone_num > last_drone_num) {
        ROS_INFO("No init before, try to init");
        finish_init = solve_with_multiple_init(last_drone_num);
        if (finish_init) {
            last_drone_num = drone_num;
            ROS_INFO("Finish init\n");
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
                est_poses_tsid_saved[sf.ts][_id] = new double[6];
            }

            if (est_poses_idts_saved[_id].find(sf.ts) == est_poses_idts_saved[_id].end()) {
                est_poses_idts_saved[_id][sf.ts] = new double[6];
            }

            memcpy(est_poses_tsid_saved[sf.ts][_id], _est_poses_tsid.at(sf.ts).at(_id), 6* sizeof(double));
            memcpy(est_poses_idts_saved[_id][sf.ts], _est_poses_tsid.at(sf.ts).at(_id), 6* sizeof(double));
        }
    }

    last_saved_est_kf_ts = sf_sld_win.back().ts;
}

unsigned int SwarmLocalizationSolver::sliding_window_size() const {
    return sf_sld_win.size();
}

CostFunction *
SwarmLocalizationSolver::_setup_cost_function_by_sf(const SwarmFrame &sf, std::map<int, int> id2poseindex, bool is_lastest_frame) const {
    //Here we will only send
    SwarmFrameError * sferror = new SwarmFrameError(sf, id2poseindex, is_lastest_frame);
    int res_num = sferror->residual_count();
    auto cost_function  = new SFErrorCost(sferror);
    int poses_num = id2poseindex.size();
    for (int i =0;i < poses_num; i ++){
        cost_function->AddParameterBlock(4);
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

    problem.AddResidualBlock(
            _setup_cost_function_by_sf(sf, id2poseindex, is_lastest_frame),
            nullptr,
            pose_state
    );
}

CostFunction *
SwarmLocalizationSolver::_setup_cost_function_by_nf_win(std::vector<NodeFrame> &nf_win, const std::map<int64_t, int> & ts2poseindex, bool is_self) const {

#ifdef ENABLE_HISTORY_COV
    int _id = nf_win.front().id;
    bool use_last_lost_cov = false;
    SwarmHorizonError * she = nullptr;
    if (finish_init && last_lost_ts_of_node.find(_id) != last_lost_ts_of_node.end()) {
        // ROS_INFO("LINE 509 : id%d %d", _id, last_lost_ts_of_node.find(_id) != last_lost_ts_of_node.end());
        use_last_lost_cov = true;
        int64_t last_lost_ts = last_lost_ts_of_node.at(_id);
        const SwarmFrame & _sf = all_sf.at(last_lost_ts);
        if (_sf.id2nodeframe.find(_id) == _sf.id2nodeframe.end()) {
            ROS_ERROR("can find id in sf; exit");
            exit(-1);
        }
        const NodeFrame & last_lost = _sf.id2nodeframe.at(_id);
        nf_win.insert(nf_win.begin(), last_lost);
        SolvedPosewithCov pcov;

        assert(est_poses_tsid.find(last_lost_ts) != est_poses_tsid.end() && "L523");
        memcpy(pcov.pose, est_poses_tsid.at(last_lost_ts).at(_id), 4*sizeof(double));
        // assert( && "L528");
        if (est_cov_tsid.at(last_lost_ts).find(_id) == est_cov_tsid.at(last_lost_ts).end()) {
            ROS_ERROR("Can't find TS %d ID %d", TSShort(last_lost_ts), _id);
            exit(-1);
        }

        auto cov = est_cov_tsid.at(last_lost_ts).at(_id);
        pcov.pos_cov.x() = sqrt(cov(0, 0));
        pcov.pos_cov.y() = sqrt(cov(1, 1));
        pcov.pos_cov.z() = sqrt(cov(2, 2));
        pcov.yaw_cov = sqrt(cov(3, 3)); 
        she = new SwarmHorizonError(nf_win, ts2poseindex, is_self, true, pcov);
    
    } else {
        she = new SwarmHorizonError(nf_win, ts2poseindex, is_self);
    }
#else
    auto she = new SwarmHorizonError(nf_win, ts2poseindex, is_self);
#endif 

    auto cost_function = new HorizonCost(she);
    int res_num = she->residual_count();

    int poses_num = nf_win.size();
    if (is_self) {
        poses_num = nf_win.size() - 1;
    }
#ifdef ENABLE_HISTORY_COV
    if (use_last_lost_cov) {
        poses_num = poses_num - 1;
    }
#endif
    for (int i =0;i < poses_num; i ++){
        cost_function->AddParameterBlock(4);
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

void SwarmLocalizationSolver::setup_problem_with_sfherror(const EstimatePosesIDTS & est_poses_idts, Problem& problem, int _id, int & count) const {
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
            pose_win.push_back(nfs[ts]);
            count ++;
            const NodeFrame & _nf = all_sf.at(ts).id2nodeframe.at(_id);
            if(_nf.is_static) {
                return;
            }
            nf_win.push_back(_nf);
            ts2poseindex[ts] = nf_win.size() - 1;

            // ROS_INFO("Add TS %d ID %d", TSShort(ts), _id);
            
        } 

    }

    if (_id == self_id) {
        //Delete last one that don't need to estimate
        pose_win.erase(pose_win.end() - 1);
        // ROS_INFO("Earse last");
        count --;
    }

    CostFunction * cf = _setup_cost_function_by_nf_win(nf_win, ts2poseindex, _id==self_id);
    if (cf != nullptr) {
        problem.AddResidualBlock(cf , nullptr, pose_win);
    }

}

double SwarmLocalizationSolver::solve_once(EstimatePoses & swarm_est_poses, EstimatePosesIDTS & est_poses_idts, bool report) {

    ros::Time t1 = ros::Time::now();
    Problem problem;


//        if (solve_count % 10 == 0)
    printf("SOLVE COUNT %d Trying to solve size %d, TS %ld\n", solve_count, sliding_window_size(), swarm_est_poses.size());

    has_new_keyframe = false;
    std::vector<std::pair<int64_t, int>> param_indexs;
    for (unsigned int i = 0; i < sf_sld_win.size(); i++ ) {
        // ROS_INFO()
        this->setup_problem_with_sferror(swarm_est_poses, problem, sf_sld_win[i], param_indexs, i==sf_sld_win.size()-1);
    }
    int verify_count = 0;
    for (int _id: all_nodes) {
        this->setup_problem_with_sfherror(est_poses_idts, problem, _id, verify_count);
    }

    ROS_INFO("Total %ld params blk, verify %d", param_indexs.size(), verify_count);


    ceres::Solver::Options options;

    //SPARSE NORMAL DOGLEG 12.5ms
    //SPARSE NORMAL 21
    //DENSE NORM DOGLEG 49.31ms
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    if (finish_init) {
        options.max_solver_time_in_seconds = MAX_SOLVER_TIME;
    }else {
       options.max_num_iterations = 200;
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

    // if (solve_count % 10 == 0)
    std::cout << "\nSize:" << sliding_window_size() << "\n" << summary.BriefReport() << " Equv cost : "
              << equv_cost << " Time : " << summary.total_time_in_seconds * 1000 << "ms\n\n\n";
#ifdef DEBUG_OUTPUT_POSES

    for (auto it : est_poses_idts) {
        auto id = it.first;
        ROS_INFO("\n\nID %d ", id);
        double* pose_last = nullptr;
        Pose pose_vo_last;
        for(auto it2 : it.second) {
            auto ts = it2.first;
            double * pose = it2.second;
            auto pose_vo = all_sf[ts].id2nodeframe[id].pose();
            auto poseest = Pose(pose, true);
            ROS_INFO("TS %ld POS %3.4f %3.4f %3.4f YAW %5.4fdeg", (ts/1000000)%1000000, pose[0], pose[1], pose[2], wrap_angle(pose[3])*57.3);
            ROS_INFO("POSVO        %3.4f %3.4f %3.4f YAW %5.4fdeg",
                     pose_vo.position.x(), pose_vo.position.y(), pose_vo.position.z(), pose_vo.yaw()*57.3);
            ROS_INFO("POSVOEST     %3.4f %3.4f %3.4f YAW %5.4fdeg",
                     poseest.position.x(), poseest.position.y(), poseest.position.z(), pose_vo.yaw()*57.3);
            if (pose_last!=nullptr) {
                Pose DposeVO = Pose::DeltaPose(pose_vo_last, pose_vo, true);
                Pose DposeEST = Pose::DeltaPose(Pose(pose_last, true), Pose(pose, true), true);
                Pose ERRVOEST = Pose::DeltaPose(DposeVO, DposeEST, true);
                ERRVOEST.position = ERRVOEST.position / VO_DRIFT_METER;
                double ang_err = ERRVOEST.yaw()/ VO_ERROR_ANGLE;
                
                ROS_WARN("ERRVOEST       %6.5f %6.5f %6.5f ANG  %3.2f",
                         ERRVOEST.position.x(), ERRVOEST.position.y(), ERRVOEST.position.z(), ang_err);

                ROS_INFO("DPOSVO         %6.5f %6.5f %3.4f YAW %5.4fdeg",
                         DposeVO.position.x(), DposeVO.position.y(), DposeVO.position.z(), DposeVO.yaw()*57.3);

                ROS_INFO("DPOSEST        %6.5f %6.5f %3.4f YAW %5.4fdeg\n\n",
                         DposeEST.position.x(), DposeEST.position.y(), DposeEST.position.z(), DposeEST.yaw()*57.3);




            }

            printf("DISTANCES ");
            for (auto itj : all_sf[ts].id2nodeframe[id].dis_map) {
                int _idj = itj.first;
                double dis = itj.second;
                Pose posj_vo = all_sf[ts].id2nodeframe[_idj].pose();
                Pose posj_est(est_poses_idts[_idj][ts], true);
                double vo_dis = (posj_vo.position - pose_vo.position).norm();
                double est_dis = (posj_est.position - Pose(pose, true).position).norm();
                printf(" DIS %4.2f VO %4.2f EST %4.2f ", dis, vo_dis, est_dis);
            }

            printf("\n");


            pose_last = pose;
            pose_vo_last = pose_vo;
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
    ROS_INFO("COV size %d %d", cov.cols(), cov.rows());
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
        printf("\nTS %d ", TSShort(_ts));
        printf("ID %d SD %4.3lf %4.3lf %4.3lf %4.3lf", _id, sqrt(cov(4*j,4*j)), 
                sqrt(cov(4*j+1,4*j+1)), 
                sqrt(cov(4*j+2,4*j+2)), 
                sqrt(cov(4*j+3,4*j+3)));
            // fflush(stdout);
        // }
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