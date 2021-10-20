#pragma once
#include <iostream>
#include <swarm_msgs/swarm_types.hpp>

struct SwarmLocalOutlierRejectionParams {

};

class SwarmLocalOutlierRejection {
    SwarmLocalOutlierRejectionParams param;
    std::vector<Swarm::LoopEdge> intra_loops;
    std::vector<Swarm::LoopEdge> inter_loops;
    std::map<int, Swarm::DroneTrajectory>  & ego_motion_trajs;
public:
    SwarmLocalOutlierRejection(const SwarmLocalOutlierRejectionParams &_param, std::map<int, Swarm::DroneTrajectory> &_ego_motion_trajs):
        param(_param), ego_motion_trajs(_ego_motion_trajs) {
    }

    std::vector<uint64_t> InterLoopOutlierRejection();
    std::vector<uint64_t> IntraLoopOutlierRejection();

    std::vector<Swarm::LoopEdge> OutlierRejectionLoopEdges(const std::vector<Swarm::LoopEdge> & available_loops);
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
