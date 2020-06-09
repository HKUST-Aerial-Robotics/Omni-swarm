#include <swarm_msgs/Pose.h>
#include <swarm_msgs/swarm_frame.h>
#include <swarm_localization/swarm_types.hpp>


struct DAHypothesis{
    int self_id = -1;
    int deteted_id = -1;
    int hypo_id = -1;
};

//This file init the system with data associaition 
class LocalizationDAInit {
    std::vector<SwarmFrame> & sf_sld_win;
public:
    LocalizationDAInit();

    bool try_data_association(const std::vector<SwarmFrame> & sf_sld_win, 
        std::map<int, int> & mapper);

    bool DFS();

};