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
    int self_id = -1;

    std::map<int, DroneTraj> ego_motions;

public:
    LocalizationDAInit(std::vector<SwarmFrame> & _sf_sld_win):
        sf_sld_win(_sf_sld_win)
    { }

    bool try_data_association(std::map<int, int> & mapper);

    bool DFS(const std::map<int, DroneTraj> est_pathes, 
        std::map<int, int> & guess, std::set<int> & unidentified) {

    }

    bool verify(std::map<int, int> & guess);

    //return 0: not observable
    //return 1: good
    //return -1: estimate failed
    int estimate_path(DroneTraj & traj, int idj, std::map<int, int> & guess, 
        const std::map<int, DroneTraj> est_pathes);
};