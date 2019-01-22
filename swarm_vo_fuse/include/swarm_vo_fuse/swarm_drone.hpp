#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>

class DroneFrame {
    int _id;
    std::map<int, float> distance;
    Eigen::Vector3d self_pos;
    Eigen::Vector3d self_vel;
    Eigen::Quaterniond self_att;
    // std::vector<>
public:
    DroneFrame()
    {

    }
};