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
#include "swarm_detection/local_pose_parameter.h"

// #define DEBUG_OUTPUT_POSES



bool SwarmLocalizationSolver::detect_outlier(const SwarmFrame &sf) const {
    //Detect if it's outlier

    // for (int i = 0; i  < s)
    return false;
}

std::vector<int> SwarmLocalizationSolver::judge_is_key_frame(const SwarmFrame &sf) {

    const std::vector<int>& _ids = sf.node_id_list;
    std::vector<int> ret(0);
    if (_ids.size() < 2)
        return ret;

//        //Temp code
//        if (_ids.size() < drone_num) {
//            return ret;
//        }

    if (sf_sld_win.empty()) {
        for (auto _id : _ids) {
            node_kf_count[_id] = 1;
        }
        return _ids;
    }


    for (auto _id : _ids) {
        if (sf_sld_win.back().HasID(_id)) {
            Eigen::Vector3d _diff = sf.position(_id) - sf_sld_win.back().position(_id);
            if (_diff.norm() > min_accept_keyframe_movement || (sf.HasDetect(_id) && _id==self_id)) {
                ret.push_back(_id);
                node_kf_count[_id] += 1;
//                    ROS_INFO("SF %ld is kf of %d: DIFF %3.2f HAS %d", sf.ts, _id, _diff.norm(), sf.HasDetect(_id));
            }
        } else {
            ret.push_back(_id);
            node_kf_count[_id] += 1;
            ROS_INFO("Last frame no id %d; Adding", _id);
        }
    }
    return ret;
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
    unsigned int i = 0;
    //Delete non keyframe first
    while (i < sf_sld_win.size() && sf_sld_win.size() > max_frame_number) {
        if (!is_frame_useful(i)) {
            delete_frame_i(i);
        } else {
            i++;
        }
    }

    while (sf_sld_win.size() > max_frame_number) {
        delete_frame_i(0);
        ROS_INFO("Clear first frame from sld win, now size %ld", sf_sld_win.size());
    }
}

void SwarmLocalizationSolver::add_as_keyframe(const SwarmFrame &sf) {
    sf_sld_win.push_back(sf);
    all_sf[sf.ts] = sf;
    EstimatePoses & est_poses = est_poses_tsid;
    EstimatePosesIDTS & est_poses2 = est_poses_idts;

    for (auto it : sf.id2nodeframe) {
        int _id = it.first;
        auto _p = new double[4];
        if (_id != self_id) {
            double noise = this->initial_random_noise;

            Pose _last;
            if (last_kf_ts > 0) {
                //Use last solve relative res, e.g init with last
                _last = Pose(est_poses_tsid[last_kf_ts][_id], true);

                Pose last_vo = all_sf[last_kf_ts].id2nodeframe[_id].pose();
                Pose now_vo = it.second.pose();
                now_vo.attitude = now_vo.attitude_yaw_only();
                last_vo.attitude = last_vo.attitude_yaw_only();

                Eigen::Isometry3d TnowVO = now_vo.to_isometry();
                Eigen::Isometry3d TlastVO = last_vo.to_isometry();

                Eigen::Isometry3d Tlast = _last.to_isometry();

                Pose transfered_now(Tlast*TlastVO.inverse()*TnowVO);


                transfered_now.to_vector_xyzyaw(_p);

            } else {
                _last.position.x() = rand_FloatRange(-noise, noise) + it.second.pose().position.x();
                _last.position.y() = rand_FloatRange(-noise, noise) + it.second.pose().position.y();
                _last.position.z() = rand_FloatRange(-noise, noise) + it.second.pose().position.z();

                _last.attitude = it.second.pose().attitude;

                _last.to_vector_xyzyaw(_p);
            }
        } else {
            Pose p = it.second.pose();
            p.to_vector_xyzyaw(_p);
            // ROS_INFO("x y z yaw:%7.6f %7.6f %7.6f Y %7.6f", _p[0], _p[1], _p[2], _p[3]*57.3, noise);
        }

        if (est_poses.find(sf.ts) == est_poses.end()) {
            est_poses[sf.ts] = std::map<int, double*>();
        }

        if(est_poses2.find(_id) == est_poses2.end()) {
            est_poses2[_id] = std::map<int64_t, double*>();
        }
        est_poses[sf.ts][_id] = _p;
        est_poses2[_id][sf.ts] = _p;
    }


    last_kf_ts = sf.ts;
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
    std::vector<int> is_kf_list = judge_is_key_frame(sf);
    if (!is_kf_list.empty()) {
        has_new_keyframe = true;
        add_as_keyframe(sf);
        ROS_INFO("New kf found, sld win size %ld", sf_sld_win.size());
    }

    if (_ids.size() > drone_num) {
        //For here the drone num increase
        drone_num = _ids.size();
    }
}


// Eigen::Vector3d predict_pos(Eigen::Vector3d pos, Eigen::e)

Pose SwarmLocalizationSolver::PredictNode(const NodeFrame & nf, bool attitude_yaw_only) const {
    if (last_saved_est_kf_ts > 0 && finish_init &&
        est_poses_tsid_saved.at(last_saved_est_kf_ts).find(nf.id)!=est_poses_tsid_saved.at(last_saved_est_kf_ts).end() ) {

        //Use last solve relative res, e.g init with last
        int _id = nf.id;
        Pose _last = Pose(est_poses_tsid_saved.at(last_saved_est_kf_ts).at(_id), true);

        Pose last_vo = all_sf.at(last_saved_est_kf_ts).id2nodeframe.at(_id).pose();
        Pose now_vo = nf.pose();

        if (attitude_yaw_only) {
            now_vo.attitude = now_vo.attitude_yaw_only();
            last_vo.attitude = last_vo.attitude_yaw_only();
        }
        Eigen::Isometry3d TnowVO = now_vo.to_isometry();
        Eigen::Isometry3d TlastVO = last_vo.to_isometry();


        Eigen::Isometry3d Tlast = _last.to_isometry();

        Pose transfered_now(Tlast*TlastVO.inverse()*TnowVO);

        return transfered_now;

    } else {
        ROS_ERROR("Can't Predict Node Pose: Not inited");
        exit(-1);
        return Pose();
    }
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
            sfs.node_poses[_id] = this->PredictNode(nf);
            //Give node velocity predict here
            sfs.node_vels[_id] = Eigen::Vector3d(0, 0, 0);
        } else {
            ROS_WARN("No id %d found in last kf", nf.id);
        }
    }

    return sfs;
}


bool SwarmLocalizationSolver::solve_with_multiple_init(int start_drone_num, int min_number, int max_number) {

    double cost = acpt_cost;
    bool cost_updated = false;

    // ROS_INFO("Try to use multiple init to solve expect cost %f", cost);

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
    if (self_id < 0
        || node_kf_count.find(self_id) == node_kf_count.end()
        || node_kf_count[self_id] < min_frame_number)
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

    if (cost_now > acpt_cost)
        finish_init = false;

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
    cost_function->SetNumResiduals(res_num);
    return cost_function;
}



void SwarmLocalizationSolver::setup_problem_with_sferror(const EstimatePoses & swarm_est_poses, Problem& problem, const SwarmFrame& sf, bool is_lastest_frame) const {
    std::vector<double*> pose_state;
    std::map<int, int> id2poseindex;
    int64_t ts = sf.ts;
    for(auto it : sf.id2nodeframe) {
        int _id = it.first;
        if (is_lastest_frame && _id == self_id) {
            continue;
        } else {
            pose_state.push_back(swarm_est_poses.at(ts).at(_id));
            id2poseindex[_id] = pose_state.size() - 1;
        }
    }

    problem.AddResidualBlock(
            _setup_cost_function_by_sf(sf, id2poseindex, is_lastest_frame),
            nullptr,
            pose_state
    );
}

CostFunction *
SwarmLocalizationSolver::_setup_cost_function_by_nf_win(const std::vector<NodeFrame> &nf_win, const std::map<int64_t, int> & ts2poseindex, bool is_self) const {
    auto she = new SwarmHorizonError(nf_win, ts2poseindex, is_self);

    auto cost_function = new HorizonCost(she);
    int res_num = she->residual_count();

    int poses_num = nf_win.size();
    if (is_self) {
        poses_num = nf_win.size() - 1;
    }
    for (int i =0;i < poses_num; i ++){
        cost_function->AddParameterBlock(4);
    }
//        ROS_INFO("SFHorizon res %d", res_num);
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
            pose_win.push_back(nfs[ts]);
            nf_win.push_back(all_sf.at(ts).id2nodeframe.at(_id));
            ts2poseindex[ts] = nf_win.size() - 1;
        } 

    }

    if (_id == self_id) {
        //Delete last one that don't need to estimate
        pose_win.erase(pose_win.end() - 1);
    }

    problem.AddResidualBlock(_setup_cost_function_by_nf_win(nf_win, ts2poseindex, _id==self_id), nullptr, pose_win);

}

double SwarmLocalizationSolver::solve_once(EstimatePoses & swarm_est_poses, EstimatePosesIDTS & est_poses_idts, bool report) {

    Problem problem;

//        if (solve_count % 10 == 0)
    printf("SOLVE COUNT %d Trying to solve size %d, poses %ld\n", solve_count, sliding_window_size(), swarm_est_poses.size());

    has_new_keyframe = false;

    for (unsigned int i = 0; i < sf_sld_win.size(); i++ ) {
        this->setup_problem_with_sferror(swarm_est_poses, problem, sf_sld_win[i], i==sf_sld_win.size()-1);
    }

    for (int _id: all_nodes) {
        this->setup_problem_with_sfherror(est_poses_idts, problem, _id);
    }


    ceres::Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 200;
    options.num_threads = thread_num;
    Solver::Summary summary;

    // options.minimizer_progress_to_stdout = true;
    // options.trust_region_strategy_type = ceres::DOGLEG;

    // std::cout << "Start solving problem" << std::endl;
    ceres::Solve(options, &problem, &summary);

    double equv_cost = 2 * summary.final_cost / sliding_window_size();

    equv_cost = equv_cost / (double) (drone_num * (drone_num - 1) / 2);
    equv_cost = sqrt(equv_cost);
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
    return equv_cost;
}
