#include "swarm_localization/localization_DA_init.hpp"
#include <ros/ros.h>

bool LocalizationDAInit::try_data_association(const std::vector<SwarmFrame> &sf_sld_win, std::map<int, int> &mapper) {
    //First we try to summarized all the UNIDENTIFIED detections
    std::set<int> unidentified;
    for (auto sf : sf_sld_win) {
        for (auto it: sf.id2nodeframe) {
            auto & _nf = it.second;
            for (auto it: _nf.detected_nodes) {
                if (it.first >= UNIDENTIFIED_MIN_ID) {
                    unidentified.insert(it.first);
                }
            }
        }
    }

    ROS_INFO("The sliding window contain %d unidentified drones", unidentified.size());

    //Secondly, we start give guess

    std::map<int, int> guess;
    if (DFS(sf_sld_win, guess, unidentified)) {
        ROS_INFO("Initial guess is OK");
    }
}