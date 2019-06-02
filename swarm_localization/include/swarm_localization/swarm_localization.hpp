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
#include "localiztion_costfunction.hpp"
#include <functional>
#include <swarm_localization/swarm_types.hpp>
#include <set>
#include "swarm_detection/local_pose_parameter.h"

typedef std::map<int, Eigen::Vector3d> ID2Vector3d;
typedef std::map<int, Eigen::Quaterniond> ID2Quat;

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace swarm;
using namespace Eigen;

float rand_FloatRange(float a, float b) {
    return ((b - a) * ((float) rand() / RAND_MAX)) + a;
}

typedef ceres::DynamicAutoDiffCostFunction<SwarmFrameError, 7>  SFErrorCost;
typedef ceres::DynamicAutoDiffCostFunction<SwarmHorizonError, 7> HorizonCost;

//Poses is dict of timestamp and then id;
//state<ts,id>
typedef std::map<int64_t, std::map<int,double*>> EstimatePoses;
typedef std::map<int, std::map<int64_t,double*>> EstimatePosesIDTS;

class SwarmLocalizationSolver {


    std::vector<SwarmFrame> sf_sld_win;
    std::map<int64_t, SwarmFrame> all_sf;
    unsigned int drone_num = 0;

    unsigned int solve_count = 0;
    const double min_accept_keyframe_movement = 0.2;

//    std::vector<double*> _swarm_est_poses;
//

    EstimatePoses swarm_poses_state;
    EstimatePosesIDTS nodeid_ts_poses;

public:
    std::set<int> all_nodes;
    unsigned int max_frame_number = 20;
    unsigned int min_frame_number = 10;
    unsigned int last_drone_num = 0;
    int self_id = -1;
    unsigned int thread_num;
    double cost_now = 0;
    double acpt_cost = 0.4;

    int last_problem_ptr = 0;
    bool finish_init = false;

    std::map<int, Vector3d> est_pos;
    std::map<int, Vector3d> est_vel;


    ros::Time last_est_time_tick = ros::Time::now();

    std::map<unsigned int, unsigned int> node_kf_count;


    bool has_new_keyframe = false;


    SwarmLocalizationSolver(int _max_frame_number, int _min_frame_number, double _acpt_cost = 0.4,
               int _thread_num = 4) :
            max_frame_number(_max_frame_number), min_frame_number(_min_frame_number),
            thread_num(_thread_num), acpt_cost(_acpt_cost) {
    }

    void init_pose(int _id, int tick, double * _p) {
        if (_id == self_id) {
            sf_sld_win[tick].id2nodeframe[_id].pose().to_vector(_p);
            return;
        }
        _p[0] = rand_FloatRange(-3, 3);
        _p[1] = rand_FloatRange(-3, 3);
        _p[2] = rand_FloatRange(0, 3);

        _p[3] = 1;
        _p[4] = 0;
        _p[5] = 0;
        _p[6] = 0;
    }

    void random_init_pose(EstimatePoses & est_poses, EstimatePosesIDTS & est_poses2, int start = 0, int end = 100) {
        for(unsigned int i = 0;i < sf_sld_win.size(); i++) {
            for (auto it : sf_sld_win[i].id2nodeframe) {
                    auto sf = sf_sld_win[i];
                    int _id = it.first;
                    auto _p = new double[7];
                    if (_id != self_id) {
                        init_pose(it.first, i, _p);
                    } else {
                        it.second.pose().to_vector(_p);
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
            }

//        ROS_INFO("EST %d poses", _est_poses.size());
    }

    bool detect_outlier(const SwarmFrame &sf) {
        //Detect if it's outlier

        // for (int i = 0; i  < s)
        return false;
    }

    std::vector<int> judge_is_key_frame(const SwarmFrame &sf) {

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
                if (_diff.norm() > min_accept_keyframe_movement || sf.HasDetect(_id)) {
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

    void delete_frame_i(int i) {
        sf_sld_win.erase(sf_sld_win.begin() + i);
    }

    bool is_frame_useful(unsigned int i) {
        for (unsigned int id : sf_sld_win[i].node_id_list) {
            if (node_kf_count[id] < min_frame_number) {
                return true;
            }
        }
        return false;
    }

    void process_frame_clear() {
        unsigned int i = 0;
        //Delete non keyframe first
        while (i < sf_sld_win.size() && sf_sld_win.size() > max_frame_number) {
            if (!is_frame_useful(i)) {
                delete_frame_i(i);
            } else {
                i++;
            }
        }
    }


    void add_new_swarm_frame(const SwarmFrame &sf) {
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
            sf_sld_win.push_back(sf);
            all_sf[sf.ts] = sf;
            ROS_INFO("New key frame found, sld win size %d", sf_sld_win.size());
        }

        if (_ids.size() > drone_num) {
            //For here the drone num increase
            drone_num = _ids.size();
        }
    }

    Eigen::Vector3d get_estimate_pos(int _id) {
        return est_pos[_id];
    }

    // Eigen::Vector3d predict_pos(Eigen::Vector3d pos, Eigen::e)

    bool PredictSwarm(const SwarmFrame &sf, SwarmFrameState & _s_state) {

        auto _ids = sf.node_id_list;
        unsigned int drone_num_now = _ids.size();

        int self_ptr = 0;
        for (unsigned int i = 0; i < _ids.size(); i++) {
            if (_ids[i] == self_id) {
                self_ptr = i;
                break;
            }
        }

        return true;


    }


    bool solve_with_multiple_init(int start_drone_num, int min_number = 5, int max_number = 10) {

        double cost = acpt_cost;
        bool cost_updated = false;

        // ROS_INFO("Try to use multiple init to solve expect cost %f", cost);

        EstimatePoses _est_poses;
        EstimatePosesIDTS _est_poses_idts;
        for (int i = 0; i < max_number; i++) {
            random_init_pose(_est_poses,  _est_poses_idts, start_drone_num, drone_num);
            double c = solve_once(_est_poses,  _est_poses_idts,  true);
            ROS_INFO("Got better cost %f", c);

            if (c < cost) {
                ROS_INFO("Got better cost %f", c);
                cost_updated = true;
                cost_now = cost = c;
                swarm_poses_state = _est_poses;
                nodeid_ts_poses = _est_poses_idts;
                if (i > min_number) {
                    return true;
                }
            }
        }

        return cost_updated;
    }

    double solve() {
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
            cost_now = solve_once(this->swarm_poses_state, this->nodeid_ts_poses, true);
        }

        if (cost_now > acpt_cost)
            finish_init = false;
        return cost_now;
    }

    inline unsigned int sliding_window_size() const {
        return sf_sld_win.size();
    }

    CostFunction *
    _setup_cost_function_by_sf(SwarmFrame &sf, std::map<int, int> id2poseindex, bool is_lastest_frame) {
        //Here we will only send
        SwarmFrameError * sferror = new SwarmFrameError(sf, id2poseindex, is_lastest_frame);
        int res_num = sferror->residual_count();
        auto cost_function  = new SFErrorCost(sferror);
        int poses_num = id2poseindex.size();
//        ROS_INFO("SF at %ld Poses num %d res num %d", sf.ts, poses_num, res_num);
        for (int i =0;i < poses_num; i ++){
            cost_function->AddParameterBlock(7);
        }
        cost_function->SetNumResiduals(res_num);
        return cost_function;
    }



    void setup_problem_with_sferror(EstimatePoses & swarm_est_poses, Problem& problem, SwarmFrame& sf, bool is_lastest_frame) {
        std::vector<double*> pose_state;
        std::map<int, int> id2poseindex;
        int64_t ts = sf.ts;
        for(auto it : sf.id2nodeframe) {
            int _id = it.first;
            if (is_lastest_frame && _id == self_id) {
                continue;
            } else {
                pose_state.push_back(swarm_est_poses[ts][_id]);
                id2poseindex[_id] = pose_state.size() - 1;
                ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                problem.AddParameterBlock(swarm_est_poses[ts][_id], 7, local_parameterization);

            }
        }

        problem.AddResidualBlock(
                _setup_cost_function_by_sf(sf, id2poseindex, is_lastest_frame),
                nullptr,
                pose_state
        );
    }

    CostFunction *
    _setup_cost_function_by_nf_win(std::vector<NodeFrame> &nf_win, std::map<int64_t, int> ts2poseindex, bool is_self) {
        auto she = new SwarmHorizonError(nf_win, ts2poseindex, is_self);

        auto cost_function = new HorizonCost(she);
        int res_num = she->residual_count();

        int poses_num = nf_win.size();
        if (is_self) {
            poses_num = nf_win.size() - 1;
        }
        for (int i =0;i < poses_num; i ++){
            cost_function->AddParameterBlock(7);
        }
//        ROS_INFO("SFHorizon res %d", res_num);
        cost_function->SetNumResiduals(res_num);
        return cost_function;
    }

    void setup_problem_with_sfherror(EstimatePosesIDTS est_poses_idts, Problem& problem, int _id) {
        if (est_poses_idts[_id].size() < 2) {
            ROS_INFO("Frame nums for id %d is to small:%ld", _id, est_poses_idts[_id].size());
            return;
        }

//        ROS_INFO("Setup problem for id %d", _id);
        std::vector<NodeFrame> nf_win;
        std::vector<double*> pose_win;
        std::map<int64_t, int> ts2poseindex;

        auto nfs = est_poses_idts[_id];
        for (auto it: nfs) {
            auto ts = it.first;
//            ROS_INFO("NODE %d TS %ld", _id, ts);
            pose_win.push_back(it.second);
            nf_win.push_back(all_sf[ts].id2nodeframe[_id]);
            ts2poseindex[ts] = nf_win.size() - 1;

//            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
//            problem.AddParameterBlock(it.second, 7, local_parameterization);
        }

        if (_id == self_id) {
            //Delete last one that don't need to estimate
            pose_win.erase(pose_win.end() - 1);
        }

        problem.AddResidualBlock(_setup_cost_function_by_nf_win(nf_win, ts2poseindex, _id==self_id), nullptr, pose_win);

    }

    double solve_once(EstimatePoses & swarm_est_poses, EstimatePosesIDTS & est_poses_idts, bool report = false) {

        Problem problem;

//        if (solve_count % 10 == 0)
        printf("TICK %d Trying to solve size %d, poses %ld\n", solve_count, sliding_window_size(), swarm_est_poses.size());

        has_new_keyframe = false;

        for (unsigned int i = 0; i < sf_sld_win.size(); i++ ) {
            this->setup_problem_with_sferror(swarm_est_poses, problem, sf_sld_win[i], i==sf_sld_win.size()-1);
        }

        for (int _id: all_nodes) {
            this->setup_problem_with_sfherror(est_poses_idts, problem, _id);
        }


        last_problem_ptr = sliding_window_size();

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.max_num_iterations = 200;
        options.num_threads = thread_num;
        Solver::Summary summary;

        // options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::DOGLEG;

        // std::cout << "Start solving problem" << std::endl;
        ceres::Solve(options, &problem, &summary);

        double equv_cost = 2 * summary.final_cost / sliding_window_size();

        equv_cost = equv_cost / (double) (drone_num * (drone_num - 1) / 2);
        equv_cost = sqrt(equv_cost);
        if (!report) {
            return equv_cost;
        }



        // if (solve_count % 10 == 0)
        std::cout << "\n\nSize:" << sliding_window_size() << "\n" << summary.BriefReport() << " Equv cost : "
                  << equv_cost << " Time : " << summary.total_time_in_seconds * 1000 << "ms\n";
        for (auto it : est_poses_idts) {
            auto id = it.first;
            ROS_INFO("\n\nID %d ", id);

            for(auto it2 : it.second) {
                auto ts = it2.first;
                double* pose = it2.second;
                auto pose_vo = all_sf[ts].id2nodeframe[id].pose();
                ROS_INFO("TS %ld POS %3.4f %3.4f %3.4f QUAT %3.2f %3.2f %3.2f %3.2f", (ts/1000000)%1000000, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);
                ROS_INFO("POSVO        %3.4f %3.4f %3.4f QUAT %3.2f %3.2f %3.2f %3.2f",
                        pose_vo.position.x(), pose_vo.position.y(), pose_vo.position.z(), pose_vo.attitude.w(), pose_vo.attitude.x(),pose_vo.attitude.y(), pose_vo.attitude.z());
            }
        }


        if (solve_count % 10 == 0)
            auto sf = sf_sld_win.back();


        solve_count++;

        return equv_cost;
    }
};
