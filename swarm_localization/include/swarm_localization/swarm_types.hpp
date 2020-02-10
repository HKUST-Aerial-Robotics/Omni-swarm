#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>
#include <swarm_msgs/Pose.h>
#include <ros/ros.h>
#include "yaml-cpp/yaml.h"
#include <exception>
#include <set>
#include <swarm_msgs/LoopConnection.h>

#define VO_DRIFT_METER 0.003 //1/100m; 2e-3 per kf
#define VO_DRIFT_METER_Z 0.005
#define VO_ERROR_ANGLE 3e-6 //3deg/1000m; average kf 0.2m, e.g 6e-4deg kf, eg 3e^-6
#define DISTANCE_MEASURE_ERROR 0.1
#define LOOP_COV 0.05
#define LOOP_YAWCOV 0.01
#define ERROR_NORMLIZED 0.01
//#define DETECTION_COV_POS 10

#define VO_DRIFT_XYZ (Eigen::Vector3d::Ones() * VO_DRIFT_METER)
using namespace Swarm;

typedef std::map<int, double> DisMap;

inline int TSShort(int64_t ts) {
    return (ts/1000000)%10000000;
}

inline int64_t TSLong(int64_t ts) {
    return (ts/1000000)%10000000000;
}


using namespace Eigen;
namespace Swarm {
    class Node {
    protected:
        bool _has_vo = false;
        bool _has_uwb = false;
        bool _has_global_pose = false;
        bool _has_global_velocity = false;
        bool _is_static = false;

        Pose global_pose;
        Eigen::Vector3d global_velocity = Vector3d(0, 0, 0);
        Vector3d anntena_pos = Vector3d(0, 0, 0);

    public:

        std::map<int, std::vector<double>> coeffs;

        int id = -1;

        bool has_odometry() const {
            return _has_vo;
        }

        bool has_uwb() const {
            return _has_uwb;
        }

        bool has_global_pose() const {
            return _has_global_pose;
        }

        bool is_static_node() const {
            return _is_static;
        }

        double to_real_distance(const double mea, int _id) const {
            assert(coeffs.find(_id) != coeffs.end() && "NO SUCH ID ON IN DISTANCE PARAMS");
            auto _coeffs = coeffs.at(_id);
            return _coeffs[0] + _coeffs[1] * mea;
        }

        Node(int _id) :
                id(_id) {

        }

        Node(int _id, const YAML::Node & config):
            id(_id){
            try {
//                ROS_INFO("Is parsing node %d", _id);
                _has_uwb = config["has_uwb"].as<bool>();
                _has_vo = config["has_vo"].as<bool>();
                _has_global_pose = config["has_global_pose"].as<bool>();
                _is_static = config["is_static"].as<bool>();
                if (_has_uwb) {
                    this->anntena_pos = Vector3d( config["anntena_pos"][0].as<double>(),
                            config["anntena_pos"][1].as<double>(),
                            config["anntena_pos"][2].as<double>());
                }

                const YAML::Node & bias_node = config["bias"];
                for(auto it=bias_node.begin();it!=bias_node.end();++it) {
                    int _node_id = it->first.as<int>();
                    std::vector<double> _coeffs = it->second.as<std::vector<double>>();
                    this->coeffs[_node_id] = _coeffs;
                }
            }
            catch (YAML::ParserException & e){
                ROS_ERROR("Error while parsing node config %d: %s, exit", _id, e.what());
                exit(-1);
            }
        }

        Pose get_global_pose() const {
            return global_pose;
        }

        Vector3d get_global_velocity() const {
            return global_velocity;
        }

        Vector3d get_anntena_pos() const {
            return anntena_pos;
        }
    };

class LoopConnection {
public:
    int64_t ts_a;
    int64_t ts_b;
    int id_a;
    int id_b;
    Pose relative_pose;
    Pose self_pose_a;
    Pose self_pose_b;
    
    LoopConnection(swarm_msgs::LoopConnection loc) {
        id_a = loc.id_a;
        id_b = loc.id_b;
        ts_a = loc.ts_a.toNSec();
        ts_b = loc.ts_b.toNSec();

        relative_pose = Pose(loc.dpos, loc.dyaw);

        self_pose_a = Pose(loc.self_pose_a);
        self_pose_b = Pose(loc.self_pose_b);
    }

    LoopConnection() {

    }
};

class NodeFrame {
    public:
        bool frame_available = false;
        bool vo_available = false;
        bool dists_available = false;
        bool corners_available = false;
        bool has_detect_relpose = false;
        bool is_static = false;
        Node *node = nullptr;
        int id = -1;

        DisMap dis_map;
        Pose self_pose;
        Eigen::Vector3d self_vel = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d global_vel = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d position_cov_to_last = VO_DRIFT_XYZ;
        double yaw_cov_to_last = VO_ERROR_ANGLE;
        std::map<int, Pose> detected_nodes;
        std::map<int, bool> enabled_detection;
        std::map<int, bool> enabled_distance;
        std::map<int, Eigen::Matrix<double, 2, 3>> detect_tan_base;

        std::map<int, Eigen::Vector3d> detected_nodes_posvar;
        std::map<int, Eigen::Vector3d> detected_nodes_angvar;

        ros::Time stamp;
        int64_t ts;
        bool is_valid = false;

        NodeFrame(Node *_node) :
                node(_node) {
            is_static = _node->is_static_node();
        }

        double to_real_distance(const double & measure, int _id) const {
            return node->to_real_distance(measure, _id);
        }

        NodeFrame() {

        }

        bool has_detected_node(int _id) const {
            return detected_nodes.find(_id) != detected_nodes.end();
        }
        
        bool has_odometry() const {
            return vo_available;
        }

        Pose pose() const {
            //If has vo, return vo position
            if (!is_static) {
                assert((node->has_odometry() && vo_available) && "Try get position non non-static via VO failed node");
            }
            

            if (vo_available) {
                return self_pose;
            } else if(is_static){
                if (node->has_global_pose()) {
                    return node->get_global_pose();
                } else {
                    //Is unknown static node, using 0, 0, 0 position
                    return Pose::Identity();
                }
            }
            assert(false && "MUST STH wrong on get pose()");
            return Pose();
        }

        Eigen::Vector3d position() const {
            return pose().pos();
        }

        double yaw() const {
            return pose().yaw();
        }

        Eigen::Quaterniond attitude(bool yaw_only = false) const {
            if (yaw_only) {
                return pose().att_yaw_only();
            } else {
                return pose().att();
            }

        }

        Eigen::Vector3d get_anntena_pos() const {
            return node->get_anntena_pos();
        }

        Eigen::Vector3d velocity() const {
            assert(!(node->has_odometry() && !vo_available) && "Try get velocity on VO failed node");
            if (vo_available) {
                return self_vel;
            } else {
                if (node->has_global_pose()) {
                    return node->get_global_velocity();
                } else {
                    //Is unknown static node, using 0, 0, 0 position

                    return Vector3d(0, 0, 0);
                }
            }
        }
    };

struct SwarmFrameState {
    std::map<int, Pose> node_poses;
    std::map<int, Eigen::Matrix4d> node_covs;
    std::map<int, Vector3d> node_vels;

    std::map<int, Pose> base_coor_poses;
    std::map<int, Eigen::Matrix4d> base_coor_covs;

};

class SwarmFrame {
    public:
        std::map<int, NodeFrame> id2nodeframe;
        std::map<int, DisMap> dis_mat;
        std::set<int> node_id_list;

        int self_id = -1;

        ros::Time stamp;
        int64_t ts;

        int swarm_size() const {
            return id2nodeframe.size();
        }

        bool has_distance_measurement(const int id) const {
            if (id2nodeframe.find(id) == id2nodeframe.end()) {
                return 0;
            }
            return id2nodeframe.at(id).node->has_uwb();
        }

        bool has_distance_measurement(const int idj, const int idi) const {
            if (dis_mat.find(idj) == dis_mat.end()) {
                return false;
            }
            return dis_mat.at(idj).find(idi) != dis_mat.at(idj).end();
        }

        bool has_node(const int _id) const{
            return (id2nodeframe.find(_id) != id2nodeframe.end());
        }

        bool has_detection_measurement(const int _id) const {
            if (id2nodeframe.find(_id) == id2nodeframe.end()) {
                return false;
            }
            if (id2nodeframe.at(_id).has_detect_relpose) {
                return true;
            }
            return false;
        }

        bool has_detection_measurement(const int _idi, const int _idj) const {
            if (id2nodeframe.find(_idi) == id2nodeframe.end()) {
                return false;
            }

            if (id2nodeframe.find(_idj) == id2nodeframe.end()) {
                return false;
            }

            if (id2nodeframe.at(_idi).detected_nodes.find(_idj) != id2nodeframe.at(_idi).detected_nodes.end()) {
                return true;
            }
            return false;
        }

        int has_detection_measurement() const {
            int total_detect = 0;
            for (auto it : id2nodeframe) {
                total_detect += it.second.detected_nodes.size();
            }

            return total_detect;
        }

        int detected_number(const int _id) const {
            if (id2nodeframe.find(_id) == id2nodeframe.end()) {
                return 0;
            }
            return id2nodeframe.at(_id).detected_nodes.size();
        }

        double distance(const int idj, const int idi) const {
            assert(has_distance_measurement(idj, idi) && "Require distance not have");

            return dis_mat.at(idj).at(idi);
        }

        bool has_odometry(const int id) const {
            if (this->has_node(id)) {
                return (id2nodeframe.at(id).vo_available);
            }
            return false;
        }

        Vector3d position(const int id) const {
            assert(id2nodeframe.find(id) != id2nodeframe.end() && "Can't find position id in frame");
            return id2nodeframe.at(id).position();
        }

        Vector3d velocity(const int id) const {
            assert(id2nodeframe.find(id) != id2nodeframe.end() && "Can't find velocity id in frame");
            return id2nodeframe.at(id).velocity();
        }

        Quaterniond attitude(const int id) const {
            assert(id2nodeframe.find(id) != id2nodeframe.end() && "Can't find attitude id in frame");
            return id2nodeframe.at(id).attitude();
        }

    };
};
