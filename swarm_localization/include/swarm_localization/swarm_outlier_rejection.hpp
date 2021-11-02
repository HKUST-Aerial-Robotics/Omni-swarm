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
    std::map<int, std::map<int, DisjointGraph>> det_pcm_graph;
    std::map<int, std::map<int, std::vector<Swarm::DroneDetection>>> all_detections;
    std::map<int, std::map<int, std::set<int64_t>>> all_detections_set_by_pair;
    std::set<int64_t> all_detections_set;
    std::map<int, std::map<int, std::set<int64_t>>> good_detections_set;
public:
    SwarmLocalOutlierRejection(const SwarmLocalOutlierRejectionParams &_param, std::map<int, Swarm::DroneTrajectory> &_ego_motion_trajs):
        param(_param), ego_motion_trajs(_ego_motion_trajs) {
    }

    std::vector<Swarm::LoopEdge> OutlierRejectionLoopEdges(const std::vector<Swarm::LoopEdge> & available_loops);
    std::vector<Swarm::LoopEdge> OutlierRejectionLoopEdgesPCM(const std::vector<Swarm::LoopEdge > & inter_loops);

    std::vector<Swarm::DroneDetection> OutlierRejectionDetections(const std::vector<Swarm::DroneDetection> & new_detections);
    void OutlierRejectionDetectionsPCM(const std::vector<Swarm::DroneDetection > & detections, int ida, int idb);

    //This should be performed in swarm_loop.
    bool check_outlier_by_odometry_consistency(const Swarm::LoopEdge & loop);
};

#include <chrono>
using namespace std::chrono; 
class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

