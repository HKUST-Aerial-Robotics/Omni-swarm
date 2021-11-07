#pragma once
#include <iostream>
#include <swarm_msgs/swarm_types.hpp>

struct SwarmLocalOutlierRejectionParams {
    bool debug_write_pcm_errors = true;
    bool debug_write_pcm_good = true;
    float pcm_thres = 0.6;
    float pcm_thres_det = 1.2;
    bool enable_pcm = false;
};

typedef std::vector<std::vector<int>> DisjointGraph;
class SwarmLocalOutlierRejection {
    SwarmLocalOutlierRejectionParams param;
    std::map<int, Swarm::DroneTrajectory>  & ego_motion_trajs;
    //Drone  ida           idb            index_det       linked dets
    std::map<int, std::map<int, DisjointGraph>> loop_pcm_graph;
    std::map<int, std::map<int, std::vector<Swarm::LoopEdge>>> all_loops;
    std::map<int, std::map<int, std::set<int64_t>>> all_loops_set_by_pair;
    std::set<int64_t> all_loops_set;
    std::map<int, std::map<int, std::set<int64_t>>> good_loops_set;
public:
    SwarmLocalOutlierRejection(const SwarmLocalOutlierRejectionParams &_param, std::map<int, Swarm::DroneTrajectory> &_ego_motion_trajs):
        param(_param), ego_motion_trajs(_ego_motion_trajs) {
    }

    std::vector<Swarm::LoopEdge> OutlierRejectionLoopEdges(const std::vector<Swarm::LoopEdge> & available_loops);
    std::vector<Swarm::LoopEdge> OutlierRejectionLoopEdgesPCM(const std::vector<Swarm::LoopEdge > & inter_loops);

    //This should be performed in swarm_loop.
    bool check_outlier_by_odometry_consistency(const Swarm::LoopEdge & loop);
};

