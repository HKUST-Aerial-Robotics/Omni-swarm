#include "swarm_localization/localization_DA_init.hpp"
#include <ros/ros.h>

using namespace std;
using namespace Eigen;
using namespace Swarm;

#define MIN_DET_THRES 0.5
#define DET_BASELINE_THRES 0.5
#define MAX_DRONE_ID 1000
//For visual initial, we limit all in 10 meter is OK
#define POSITION_LIM 30
#define DFS_DEBUG_OUTPUT

LocalizationDAInit::LocalizationDAInit(int _self_id, const std::map<int, Swarm::DroneTrajectory> & _egomotions,
        const std::map<int, Swarm::DroneTrajectory> & _est_keyframes,
        std::vector<Swarm::LoopEdge> & _drone_dets_6d, double _accept_thres):
        self_id(_self_id),
        drone_dets_6d(_drone_dets_6d),
        accept_thres(_accept_thres),
        ego_motions(_egomotions),
        est_keyframes(_est_keyframes) {
    for (auto & traj : _egomotions) { 
        available_nodes.push_back(traj.first);
    }
}

bool LocalizationDAInit::try_data_association(std::map<int, int> &mapper) {
    ROS_INFO("Trying to initialize with data associaition...");
    //First we try to summarized all the UNIDENTIFIED detections
    std::set<int> unidentified;
    std::map<int, Swarm::LoopEdge> raw_detectors_by_uniden;
    for (auto & det: drone_dets_6d) {
        if (det.id_b >= MAX_DRONE_ID && ego_motions.find(det.id_a) != ego_motions.end()) {
            unidentified.insert(det.id_b);
            raw_detectors_by_uniden[det.id_b] = det;
            uniden_detector[det.id_b] = det.id_a;
            drone_dets_pair[det.id_a][det.id_b].emplace_back(det);
            drone_dets_by_pair[det.id_b][det.id_a].emplace_back(det);
        }
    }

    ROS_INFO("The drone_detections contain %ld unidentified drones", unidentified.size());

    if (unidentified.size() == 0) {
        return false;
    }

    //Secondly, we start give guess

    std::map<int, int> guess;
    std::map<int, Swarm::Pose> est_poses_t0;
    if (ego_motions.find(self_id) == ego_motions.end()) {
        return false;
    }
    TsType t0 = 0;
    for (auto & p : ego_motions) {
        auto _t0 = p.second.ts_by_index(0);
        if (_t0 > t0) {
            t0 = _t0;
        }
    }

    est_poses_t0[self_id] = ego_motions.at(self_id).pose_by_appro_ts(t0);

    //Update known poses here.
    for (auto & p : est_keyframes) {
        int _id = p.first;
        TsType t1;
        auto pose_t1 = p.second.pose_by_appro_ts(t0, t1); //Est pose at t1
        auto rp_t1tot0 = ego_motions.at(_id).get_relative_pose_by_appro_ts(t1, t0);
        auto pose_est_t0 = pose_t1 * rp_t1tot0.first;
        est_poses_t0[_id] = pose_est_t0;
    }

    auto ret = DFS(est_poses_t0, guess, unidentified, raw_detectors_by_uniden, t0);
    if (ret.first) {
        ROS_INFO("DFS guess is OK cost %f the assoication", ret.second);
        for (auto it : guess) {
            printf("%d:%d ", it.first, it.second);
        }
        printf("\n");
        for (auto & det: drone_dets_6d) {
            if (det.id_b >=MAX_DRONE_ID) {
                det.id_b = guess.at(det.id_b);
            }
        }

        return true;
    }

    return false;
}

double LocalizationDAInit::verify(int new_undenified, const std::map<int, Swarm::Pose> & est_poses, const std::map<int, int> & guess, TsType t0) const {
    //First we assume all static
    //Ignore verify now
    int new_id = guess.at(new_undenified);
    double max_dis = 0;
    if (drone_dets_pair.find(new_id) == drone_dets_pair.end()) {
        return 0.0; //Return -1 so that we need verification
    }

    for (auto dets: drone_dets_pair.at(new_id)) {
        int target_id = dets.first;
        auto edge = dets.second.at(0);
        if (target_id >= MAX_DRONE_ID) { //Target is unidentified
            if (guess.find(target_id) != guess.end()) {
                target_id = guess.at(target_id);
            }
        }
        
        //Verify the target's pose
        if (est_poses.find(target_id) != est_poses.end()) {
            auto est_pose_target = est_poses.at(target_id);
            //Compare with current estimation
            auto _est_pose_target = est_poses.at(new_id)*edge.relative_pose;
            auto diff = Swarm::Pose::DeltaPose(est_pose_target, _est_pose_target).log_map();
            auto dis = Swarm::computeSquaredMahalanobisDistance(diff, edge.get_covariance());
            if (dis > max_dis) {
                dis = max_dis;
            }
        }
    }

    return max_dis;
}

double LocalizationDAInit::verify(const std::map<int, Swarm::Pose> & est_pathes, const std::map<int, int> & guess, TsType t0) const {
    //Here we verify each drone_dets_pair
    double max_dis = 0;

    for (auto it : drone_dets_pair) {
        int detector_id = it.first;
        for (auto it2 : it.second) {
            int unidentified_target = it2.first;
            int target_id = guess.at(unidentified_target);
            if (target_id >= 0) {
                auto edge = it2.second[0];
                auto est_relative_pose = Swarm::Pose::DeltaPose(est_pathes.at(detector_id), est_pathes.at(target_id));
                auto diff = Swarm::Pose::DeltaPose(est_relative_pose, edge.relative_pose).log_map();
                auto dis = Swarm::computeSquaredMahalanobisDistance(diff, edge.get_covariance());
                if (dis > max_dis) {
                    dis = max_dis;
                }
            }
        }
    }

    return max_dis;
}

std::pair<bool, double> LocalizationDAInit::DFS(std::map<int, Swarm::Pose> & est_poses_t0, std::map<int, int> & guess, const std::set<int> & unidentified, const std::map<int, Swarm::LoopEdge> & raw_dets,
    TsType t0) {
#ifdef DFS_DEBUG_OUTPUT
    printf("\nDFS Unidentified num %ld guess ", unidentified.size());
    
    for (auto it : guess) {
        printf("%d:%d ", it.first, it.second);
    }

    printf("\n");
#endif

    if (unidentified.size() == 0) {
        double cost = verify(est_poses_t0, guess, t0);
        if (cost >= 0 && cost < accept_thres && est_poses_t0.size() == available_nodes.size()) {
#ifdef DFS_DEBUG_OUTPUT
            ROS_INFO("Guess verified, final cost %f, return.", cost);
#endif
            return make_pair(true, cost);
        } else {
#ifdef DFS_DEBUG_OUTPUT
            ROS_INFO("Guess failed, final cost %f, return.", cost);
#endif
            return make_pair(false, -1);
        }
    }

    for (auto _uniden : unidentified) {
        auto uniden_det = uniden_detector[_uniden];
        if (est_poses_t0.find(uniden_det) == est_poses_t0.end() || guess.find(_uniden) != guess.end()) {
            //Choose the drones which it's detector has been identified.
            continue;
        }
        std::map<int, Swarm::Pose> best_poses_t0;
        std::map<int, int> best_guess;
        double best_cost = 1000000;
        auto cov = raw_dets.at(_uniden).get_covariance();

        for (size_t i = 0; i <=available_nodes.size(); i++) {
            int new_id = -1;
            if (i < available_nodes.size()) {
                new_id = available_nodes[i];
            }

            //This new id must not be the detector drone itself
            if (uniden_detector[_uniden] == new_id || ego_motions.find(new_id) == ego_motions.end() || 
                ego_motions.find(raw_dets.at(_uniden).id_a) == ego_motions.end()) {
                #ifdef DFS_DEBUG_OUTPUT
                    ROS_INFO("_uniden %d new_id %d detector %d ego_new_id %d ego_id_a %d", _uniden, new_id,uniden_detector[_uniden], !(ego_motions.find(new_id) == ego_motions.end()),
                        !(ego_motions.find(raw_dets.at(_uniden).id_a) == ego_motions.end()));
                #endif

                continue;
            }

            //We need to check if other drones detect by the detector has been guess with this new id
            int detector = uniden_detector[_uniden];
#ifdef DFS_DEBUG_OUTPUT
            ROS_INFO("Try %d as %d", _uniden, new_id);
#endif

            //Here we start search this guess
            std::map<int, int> new_guess(guess);
            new_guess[_uniden] = new_id;
            std::set<int> this_unidentified(unidentified);
            this_unidentified.erase(_uniden);

            std::map<int, Swarm::Pose> new_est_poses_t0(est_poses_t0);

#ifdef DFS_DEBUG_OUTPUT
            printf("New guess ");
            for (auto it : new_guess) {
                printf("%d:%d ", it.first, it.second);
            }
            printf("\n");
#endif
            //We will try to estimate this position and verify it.
            if (new_id>0) {
                auto pose_est = estimate_path(_uniden, new_guess, new_est_poses_t0, t0);
                double dis = 0;
                if (new_est_poses_t0.find(new_id) == new_est_poses_t0.end() || 
                        (dis = Swarm::Pose::MahalanobisDistance(new_est_poses_t0.at(new_id), pose_est, cov))  <  accept_thres ) {
                    new_est_poses_t0[new_id]  = pose_est;
#ifdef DFS_DEBUG_OUTPUT
                    printf("Estimate path succ: %3.3f.\n", dis);
#endif
                } else {
#ifdef DFS_DEBUG_OUTPUT
                    auto logmap = Swarm::Pose::DeltaPose(new_est_poses_t0.at(new_id), pose_est).log_map();
                    printf("Estimate path failed: %3.3f %s %s. logmap [%3.2e,%3.1e,%3.1e,%3.1e,%3.1e,%3.1e] cov [%3.1e,%3.1e,%3.1e,%3.1e].\n", 
                        dis, new_est_poses_t0.at(new_id).tostr().c_str(), pose_est.tostr().c_str(), 
                        logmap(0), logmap(1), logmap(2), logmap(3), logmap(4), logmap(5), 
                        cov(0, 0), cov(1, 1), cov(2, 2), cov(5, 5));
#endif
                    continue;
                }
            }


            auto result = DFS(new_est_poses_t0, new_guess, this_unidentified, raw_dets, t0);
            printf("DFS with new guess return %d %f\n", result.first, result.second);

            if (result.first && result.second < best_cost) {
                //Here we assume only one result
                best_cost = result.second;
                best_guess = new_guess;
                best_poses_t0 = new_est_poses_t0;
            }
        }

        if(best_cost < 1000000) {
            guess = best_guess;
            est_poses_t0 = best_poses_t0;
            return make_pair(true, best_cost); //So we only use the first element of uninitialized
        }
    }

    //No good result, return false
    return make_pair(false, -1);
}

//We get the pose by t0
Swarm::Pose LocalizationDAInit::estimate_path(int _uniden, map<int, int> & guess, const map<int, Swarm::Pose> est_poses_t0, TsType t0) const {
    //Assume static now
    int new_id = guess.at(_uniden);
    auto it = drone_dets_by_pair.at(_uniden).begin();
    auto det_id_a = it->first;
    auto edge = it->second[0]; //Only use first detection
    Swarm::Pose odoma_0_to_1 = ego_motions.at(edge.id_a).get_relative_pose_by_appro_ts(t0, edge.ts_a).first;
    Swarm::Pose odomb_1_to_0 = ego_motions.at(new_id).get_relative_pose_by_appro_ts(edge.ts_b, t0).first;
    Swarm::Pose pose = est_poses_t0.at(edge.id_a)*odoma_0_to_1*edge.relative_pose*odomb_1_to_0;
    return pose;
}

double LocalizationDAInit::verify_with_measurements(const vector<pair<Pose, Vector3d>> & dets, 
    const vector<pair<Eigen::Vector3d, double>> &diss, const Eigen::Vector3d &point_3d, const std::map<int, Swarm::LoopEdge> & raw_dets, TsType t0) {
    double error = 0;

    //Also verify with distance
    for (auto it : diss) {
        auto distance1 = (point_3d - it.first).norm();
        auto distance2 = it.second;

        // std::cout << "Distance " << distance1 << " Distance2 " << distance2 << std::endl;

        if (abs(distance1 - distance2) > accept_distance_thres) {
            return -1;
        }

        error = error + abs(distance1 - distance2);
    }

    return error;
}