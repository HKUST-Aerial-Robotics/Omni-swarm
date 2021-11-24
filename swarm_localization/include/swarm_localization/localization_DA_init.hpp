#include <swarm_msgs/Pose.h>
#include <swarm_msgs/swarm_frame.h>
#include <swarm_msgs/swarm_types.hpp>

struct DAHypothesis{
    int self_id = -1;
    int deteted_id = -1;
    int hypo_id = -1;
};

//This file init the system with data associaition 
class LocalizationDAInit {
    int self_id = -1;

    std::map<int, Swarm::DroneTrajectory> ego_motions;
    std::map<int, Swarm::DroneTrajectory> est_keyframes;
    std::vector<Swarm::LoopEdge> & drone_dets_6d;
    std::map<int, std::map<int, std::vector<Swarm::LoopEdge>>> drone_dets_pair;
    std::map<int, std::map<int, std::vector<Swarm::LoopEdge>>> drone_dets_by_pair;

    std::vector<int> available_nodes;

    std::map<int, std::set<int>> detected_set;
    
    //The detector of the unidentified id
    //first is the unidentified id, second is the detector
    std::map<int, int> uniden_detector;

    double accept_thres = 0.1;
    double accept_distance_thres = 0.3;
    
public:
    LocalizationDAInit(int _self_id, const std::map<int, Swarm::DroneTrajectory> & _egomotions, 
        const std::map<int, Swarm::DroneTrajectory> & _est_keyframes,
        std::vector<Swarm::LoopEdge> & _drone_dets_6d, double _accept_thres);

    bool try_data_association(std::map<int, int> & mapper);

private:

    double verify_with_measurements(const std::vector<std::pair<Swarm::Pose, Eigen::Vector3d>> & dets, 
        const std::vector<std::pair<Eigen::Vector3d, double>> &diss, const Eigen::Vector3d &point_3d, const std::map<int, Swarm::LoopEdge> & raw_dets, TsType t0);
    
    std::pair<bool, double> DFS(std::map<int, Swarm::Pose> & est_pathes, std::map<int, int> & guess, const std::set<int> & unidentified, const std::map<int, Swarm::LoopEdge> & raw_dets, TsType t0);

    double verify(int new_undenified, const std::map<int, Swarm::Pose> & est_pathes, const std::map<int, int> & guess, TsType t0) const;
    double verify(const std::map<int, Swarm::Pose> & est_pathes, const std::map<int, int> & guess, TsType t0) const;

    Swarm::Pose estimate_path(int idj, std::map<int, int> & guess, const std::map<int, Swarm::Pose> est_pathes, TsType t0) const;
};