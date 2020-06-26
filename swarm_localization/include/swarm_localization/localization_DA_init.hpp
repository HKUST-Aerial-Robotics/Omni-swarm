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
    //first is the unidentified id, second is the detector
    std::map<int, int> uniden_detector;

    double triangulate_accept_thres = 0.1;

    double accept_angular_thres = 0.3;
    double accept_distance_thres = 0.5;

public:
    LocalizationDAInit(std::vector<SwarmFrame> & _sf_sld_win, double _triangulate_accept_thres);

    bool try_data_association(std::map<int, int> & mapper);

private:

    double verify_with_measurements(const std::vector<std::pair<Pose, Eigen::Vector3d>> & dets, 
        const std::vector<std::pair<Eigen::Vector3d, double>> &diss, const Eigen::Vector3d &point_3d);
    
    std::pair<bool, double> DFS(std::map<int, DroneTraj> & est_pathes, std::map<int, int> & guess, const std::set<int> & unidentified);

    bool verify(const std::map<int, DroneTraj> & est_pathes, const std::map<int, int> & guess);

    double estimate_pathes(std::map<int, DroneTraj> & est_pathes, std::map<int, int> & guess);

    bool check_guess_has_assign_id(std::map<int, int> & guess, int detector, int _new_id);

    //return 0, _: not observable
    //return 1, cost: good and the cost
    //return -1, 0: estimate failed
    std::pair<int, double> estimate_path(DroneTraj & traj, int idj, std::map<int, int> & guess, 
        const std::map<int, DroneTraj> est_pathes);
};