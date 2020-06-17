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

    std::set<int> available_nodes;

    std::map<int, std::set<int>> detected_set;
    
    //The detector of the unidentified id
    std::map<int, int> uniden_detector;

    double triangulate_accept_thres = 0.1;

public:
    LocalizationDAInit(std::vector<SwarmFrame> & _sf_sld_win, double _triangulate_accept_thres):
        sf_sld_win(_sf_sld_win), triangulate_accept_thres(_triangulate_accept_thres)
    { }

    bool try_data_association(std::map<int, int> & mapper);

private:
    bool DFS(std::map<int, DroneTraj> & est_pathes, std::map<int, int> & guess, std::set<int> & unidentified);

    bool verify(const std::map<int, DroneTraj> & est_pathes, const std::map<int, int> & guess);

    int estimate_pathes(std::map<int, DroneTraj> & est_pathes, std::map<int, int> & guess);

    //return 0: not observable
    //return 1: good
    //return -1: estimate failed
    int estimate_path(DroneTraj & traj, int idj, std::map<int, int> & guess, 
        const std::map<int, DroneTraj> est_pathes);
};