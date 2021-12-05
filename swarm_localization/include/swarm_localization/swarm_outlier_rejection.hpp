#pragma once
#include <iostream>
#include <swarm_msgs/swarm_types.hpp>

struct SwarmLocalOutlierRejectionParams {
    bool debug_write_pcm_errors = true;
    bool debug_write_debug = true;
    bool debug_write_pcm_good = true;
    float pcm_thres = 0.6;
    bool enable_pcm = false;
};

typedef std::vector<std::vector<int>> DisjointGraph;
class SwarmLocalOutlierRejection {
    SwarmLocalOutlierRejectionParams param;
    std::map<int, Swarm::DroneTrajectory>  & ego_motion_trajs;
    //Drone  ida           idb            index_det       linked dets
    std::map<int, std::map<int, DisjointGraph>> loop_pcm_graph;
    std::map<int, std::map<int, std::vector<Swarm::LoopEdge>>> all_loops;
    std::set<int64_t> all_loops_set;
    std::map<int, std::map<int, std::set<int64_t>>> all_loops_set_by_pair;
public:
    std::map<int, std::map<int, std::set<int64_t>>> good_loops_set;
    std::map<int64_t, Swarm::LoopEdge> all_loop_map;
    SwarmLocalOutlierRejection(const SwarmLocalOutlierRejectionParams &_param, std::map<int, Swarm::DroneTrajectory> &_ego_motion_trajs);

    std::vector<Swarm::LoopEdge> OutlierRejectionLoopEdges(const std::vector<Swarm::LoopEdge> & available_loops);
    void OutlierRejectionLoopEdgesPCM(const std::vector<Swarm::LoopEdge > & inter_loops, int id_a, int id_b);

    //This should be performed in swarm_loop.
    bool check_outlier_by_odometry_consistency(const Swarm::LoopEdge & loop);
    
    const std::vector<int64_t> good_loops() const {
        std::vector<int64_t> ret;
        for (auto & it1: good_loops_set) {
            for (auto & it2: it1.second) {
                for (auto it3: it2.second) {
                    ret.push_back(it3);
                }
            }
        }
        return ret;
    }
};

