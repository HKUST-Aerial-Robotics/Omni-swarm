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
#include <swarm_msgs/node_detected_xyzyaw.h>

#define VO_DRIFT_METER 0.003 //1/100m; 2e-3 per kf
#define VO_DRIFT_METER_Z 0.005
#define VO_ERROR_ANGLE 3e-6 //3deg/1000m; average kf 0.2m, e.g 6e-4deg kf, eg 3e^-6
#define DISTANCE_MEASURE_ERROR 0.1
#define LOOP_COV 0.05
#define LOOP_YAWCOV 0.01
#define ERROR_NORMLIZED 0.01
#define UNIDENTIFIED_MIN_ID 1000
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


inline Eigen::Matrix<double, 2, 3> tangent_base_for_unit_detect(const Eigen::Vector3d & pts_j) {
    Eigen::Matrix<double, 2, 3> tangent_base;
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pts_j.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();

    return tangent_base;
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

typedef std::tuple<int64_t, int64_t, int, int> GeneralMeasurement2DronesKey;

class GeneralMeasurement2Drones {
public:
    int64_t ts_a;
    int64_t ts_b;
    ros::Time stamp_a;
    ros::Time stamp_b;
    int id_a;
    int id_b;
    Pose self_pose_a;
    Pose self_pose_b;
    int res_count = 0;
    enum { 
        Loop,
        Detection
    } meaturement_type;
    GeneralMeasurement2DronesKey key() {
        return GeneralMeasurement2DronesKey(ts_a, ts_b, id_a, id_b);
    }
};

class LoopConnection: public GeneralMeasurement2Drones {
public:
    Pose relative_pose;
    LoopConnection(swarm_msgs::LoopConnection loc) {
        id_a = loc.id_a;
        id_b = loc.id_b;
        ts_a = loc.ts_a.toNSec();
        ts_b = loc.ts_b.toNSec();

        stamp_a = loc.ts_a;
        stamp_b = loc.ts_b;

        relative_pose = Pose(loc.dpos, loc.dyaw);

        self_pose_a = Pose(loc.self_pose_a);
        self_pose_b = Pose(loc.self_pose_b);
        meaturement_type = Loop;
        res_count = 4;
    }

    LoopConnection(const LoopConnection &loc) {
        id_a = loc.id_a;
        id_b = loc.id_b;
        ts_a = loc.ts_a;
        ts_b = loc.ts_b;

        stamp_a = loc.stamp_a;
        stamp_b = loc.stamp_b;

        relative_pose = loc.relative_pose;

        self_pose_a = loc.self_pose_a;
        self_pose_b = loc.self_pose_b;
        meaturement_type = Loop;
        res_count = 4;
    }
    
    LoopConnection() {
        meaturement_type = Loop;
        res_count = 0;
    }
};

class DroneDetection: public GeneralMeasurement2Drones {

public:
    Eigen::Matrix<double, 2, 3> detect_tan_base;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    double inv_dep = 0;
    double probaility = 0;

    bool enable_depth = false;
    
    Pose dpose_self_a;
    Pose dpose_self_b;
    
    DroneDetection(const swarm_msgs::node_detected_xyzyaw & nd, bool _enable_depth = true) {
        id_a = nd.self_drone_id;
        id_b = nd.remote_drone_id;
        ts_a = nd.header.stamp.toNSec();
        ts_b = nd.header.stamp.toNSec();

        stamp_a = nd.header.stamp;
        stamp_b = nd.header.stamp;

        probaility = nd.probaility;

        self_pose_a = Pose(nd.local_pose_self);
        self_pose_b = Pose(nd.local_pose_remote);

        inv_dep = nd.inv_dep;
        p = Eigen::Vector3d(nd.dpos.x, nd.dpos.y, nd.dpos.z);
        p.normalize();
        meaturement_type = Detection;

        if (_enable_depth && nd.enable_scale) {
            enable_depth = true;
            res_count = 3;
        } else {
            enable_depth = false;
            res_count = 2;
        }


        detect_tan_base = tangent_base_for_unit_detect(p);
    }


    DroneDetection(const DroneDetection & dronedet) {
        id_a = dronedet.id_a;
        id_b = dronedet.id_b;
        ts_a = dronedet.ts_a;
        ts_b = dronedet.ts_b;

        stamp_a = dronedet.stamp_a;
        stamp_b = dronedet.stamp_b;

        probaility = dronedet.probaility;

        self_pose_a = dronedet.self_pose_a;
        self_pose_b = dronedet.self_pose_b;

        inv_dep = dronedet.inv_dep;
        p = dronedet.p;
        p.normalize();
        meaturement_type = Detection;

        enable_depth = dronedet.enable_depth;
        res_count = dronedet.res_count;

        detect_tan_base = dronedet.detect_tan_base;
    }

    DroneDetection() {
        meaturement_type = Detection;
        res_count = 0;
    }
};

typedef std::vector<std::pair<int64_t, Pose>> DroneTraj;

class NodeFrame {
    public:
        bool frame_available = false;
        bool vo_available = false;
        bool dists_available = false;
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
