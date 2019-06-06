#include <iostream>
#include "glog/logging.h"
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include <vector>
#include "ros/ros.h"
#include <swarm_msgs/swarm_frame.h>
#include <swarm_msgs/node_frame.h>
#include <algorithm>
#include <set>
#include <map>
#include <ctime>
#include <thread>
#include <unistd.h>
#include "swarm_localization/swarm_localization_solver.hpp"
#include "swarm_msgs/swarm_fused.h"
#include "swarm_msgs/swarm_fused_relative.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include "yaml-cpp/yaml.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <swarm_detection/swarm_detect_types.h>


using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace Eigen;
using namespace nav_msgs;
using namespace swarm_msgs;


class SwarmLocalizationNode {

    void add_drone_id(int _id) {
        this->remote_ids_arr.push_back(_id);
        this->remote_ids_set.insert(_id);
        this->ids_index_in_arr[_id] = this->remote_ids_arr.size() - 1;
    }

    bool has_this_drone(int _id) {
        return remote_ids_set.find(_id) != remote_ids_set.end();
    }


    corner_array CA_from_msg(const std::vector<swarm_msgs::armarker_detected> &cam0_markers) const {
        corner_array ca;
        for (const swarm_msgs::armarker_detected &ad : cam0_markers) {
            for (const swarm_msgs::armarker_corner &_ac : ad.corner_detected) {
                MarkerCornerObservsed mco;

                mco.corner_no = _ac.corner_id;

                mco.observed_point.x() = _ac.x;
                mco.observed_point.y() = _ac.y;
                mco.marker = all_ar_markers.at(_ac.marker_id);

                ca.push_back(mco);
            }
        }
        return ca;
    }

    NodeFrame node_frame_from_msg(const swarm_msgs::node_frame &_nf) const {

        //TODO: Deal with global pose
        if (all_node_defs.find(_nf.id) == all_node_defs.end()) {
            ROS_ERROR("No such node %d", _nf.id);
            exit(-1);
        }
        NodeFrame nf(all_node_defs.at(_nf.id));
        nf.stamp = _nf.header.stamp;
        nf.ts = nf.stamp.toNSec();
        nf.frame_available = true;
        nf.vo_available = _nf.vo_available;
        nf.dists_available = !_nf.dismap_ids.empty();
        nf.id = _nf.id;

        assert(_nf.dismap_ids.size() == _nf.dismap_dists.size() && "Dismap ids and distance must equal size");

        for (unsigned int i = 0; i < _nf.dismap_ids.size(); i++) {
            nf.dis_map[_nf.dismap_ids[i]] = _nf.dismap_dists[i];
        }

        nf.corner_by_cams.push_back(CA_from_msg(_nf.cam0_markers));
        nf.corner_by_cams.push_back(CA_from_msg(_nf.cam1_markers));

        if (nf.vo_available) {
            nf.self_pose = Pose(_nf.odometry.pose.pose);

            nf.self_vel.x() = _nf.odometry.twist.twist.linear.x;
            nf.self_vel.y() = _nf.odometry.twist.twist.linear.y;
            nf.self_vel.z() = _nf.odometry.twist.twist.linear.z;
        } else {
            if (nf.node->HasVO()) {
                ROS_WARN("Node %d invalid: No vo now", _nf.id);
            }
            nf.is_valid = false;
        }

        for (auto nd: _nf.detected.detected_nodes) {
            nf.detected_nodes[nd.remote_drone_id] = Pose(nd.relpose.pose);
            auto cov = nd.relpose.covariance;
            nf.detected_nodes_poscov[nd.remote_drone_id] = 
                Eigen::Vector3d(cov[0], cov[6+1], cov[2*6+2]);

            nf.detected_nodes_angcov[nd.remote_drone_id] = 
                Eigen::Vector3d(cov[3*6+3], cov[4*6+4], cov[5*6+5]);
            nf.has_detect_relpose = true;
        }
        return nf;
    }

    SwarmFrame swarm_frame_from_msg(const swarm_msgs::swarm_frame &_sf) const {
        SwarmFrame sf;

        sf.stamp = _sf.header.stamp;
        sf.ts = sf.stamp.toNSec();
        sf.self_id = _sf.self_id;

        for (const swarm_msgs::node_frame &_nf: _sf.node_frames) {
            sf.node_id_list.push_back(_nf.id);
        }

        for (auto it : sf.id2nodeframe) {
            for (auto it_d :it.second.detected_nodes) {
                sf.id2nodeframe[it_d.first].has_detect_relpose = true;
                sf.id2nodeframe[it.first].has_detect_relpose = true;
            }
        }

        for (const swarm_msgs::node_frame &_nf: _sf.node_frames) {
            sf.id2nodeframe[_nf.id] = node_frame_from_msg(_nf);
            sf.dis_mat[_nf.id] = sf.id2nodeframe[_nf.id].dis_map;
        }
        return sf;
    }

    double t_last = 0;
protected:
    void on_swarmframe_recv(const swarm_msgs::swarm_frame &_sf) {
        //TODO:Fix when nodeframe invaild!!!!
//        ROS_INFO("Recv swarm frame with %ld nodes", _sf.node_frames.size());
        SwarmFrame sf = swarm_frame_from_msg(_sf);

        int _self_id = _sf.self_id;
        frame_id = "world";

        swarm_localization_solver->self_id = _self_id;

        if (remote_ids_arr.empty()) {
            //This is first time of receive data
            this->self_id = _self_id;
            swarm_localization_solver->self_id = self_id;
            ROS_INFO("self id %d", self_id);
            add_drone_id(self_id);
        }

        for (int _id: sf.node_id_list) {
            if (!has_this_drone(_id))
                add_drone_id(_id);
        }


        //


        double t_now = _sf.header.stamp.toSec();

        // printf("Tnow %f\n", t_now);
        
        if (t_now - t_last > 1 / force_freq) {
            swarm_localization_solver->add_new_swarm_frame(sf);
            std_msgs::Float32 cost;
            cost.data = this->swarm_localization_solver->solve();
            t_last = t_now;
            if (cost.data > 0)
                solving_cost_pub.publish(cost);
        }
    }

    void pub_posevel_id(unsigned int id, const Pose & pose, const Eigen::Vector3d vel, ros::Time stamp) {
        Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "world";
        odom.pose.pose = pose.to_ros_pose();
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        pub_odom_id(id, odom);
    }

    void pub_odom_id(unsigned int id, const Odometry &odom) {
        if (remote_drone_odom_pubs.find(id) == remote_drone_odom_pubs.end()) {
            char name[100] = {0};
            sprintf(name, "/swarm_drones/est_drone_%d_odom", id);
            remote_drone_odom_pubs[id] = nh.advertise<Odometry>(name, 1);
        }

        auto pub = remote_drone_odom_pubs[id];

        pub.publish(odom);
    }

    float force_freq = 10;
    ros::NodeHandle &nh;

private:

    ros::Subscriber recv_sf_est, recv_sf_predict;
    ros::Subscriber recv_drone_odom_now;
    ros::Publisher fused_drone_data_pub, solving_cost_pub, fused_drone_rel_data_pub;

    std::string frame_id = "";

    std::map<int, ros::Publisher> remote_drone_odom_pubs;
    SwarmLocalizationSolver *swarm_localization_solver = nullptr;

    std::vector<int> remote_ids_arr;
    std::set<int> remote_ids_set;
    std::map<int, int> ids_index_in_arr;
    std::map<int, Node *> all_node_defs;
    std::map<int, DroneMarker *> all_ar_markers;


    ros::Timer timer;


    int self_id = -1;

    void load_nodes_from_file(const std::string &path) {
        try {
            ROS_INFO("Loading swarmconfig from %s", path.c_str());
            YAML::Node nodes_config = YAML::LoadFile(path)["nodes"];
            for(YAML::iterator it=nodes_config.begin();it!=nodes_config.end();++it) {
                    int node_id = it->first.as<int>();
                    const YAML::Node & _node_config = it->second;
                    ROS_INFO("Parsing node %d", node_id);
                    Node *new_node = new Node(node_id, _node_config);
                    all_node_defs[node_id] = new_node;
                    ROS_INFO("NODE %d static:%d vo %d uwb %d armarker %d",
                             new_node->id,
                             new_node->IsStatic(),
                             new_node->HasVO(),
                             new_node->HasUWB(),
                             new_node->HasArmarker()
                    );
                }

        } catch (std::exception & e) {
            ROS_ERROR("Error while parsing config file:%s, exit",e.what());
            exit(-1);
        }

    }

    void predict_swarm(const swarm_frame &_sf) {
        if (swarm_localization_solver->CanPredictSwarm()) {
            SwarmFrame sf = swarm_frame_from_msg(_sf);
            SwarmFrameState _sfs = swarm_localization_solver->PredictSwarm(sf);
            for (auto it: _sfs.node_poses) {
                this->pub_posevel_id(it.first, it.second, _sfs.node_vels[it.first], sf.stamp);
            }
        } else {
            ROS_WARN_THROTTLE(1.0, "Unable to predict swarm");
        }
    }

public:
    SwarmLocalizationNode(ros::NodeHandle &_nh) :
            nh(_nh) {
        recv_sf_est = nh.subscribe("/swarm_drones/swarm_frame", 10,
                                          &SwarmLocalizationNode::on_swarmframe_recv, this,
                                          ros::TransportHints().tcpNoDelay());
        
        recv_sf_predict = nh.subscribe("/swarm_drones/swarm_frame_predict", 1,
                                          &SwarmLocalizationNode::predict_swarm, this,
                                          ros::TransportHints().tcpNoDelay());
        
        int frame_num = 0, thread_num, min_frame_num;
        float acpt_cost = 0.4;

        std::string swarm_node_config;


        nh.param<int>("max_keyframe_num", frame_num, 100);
        nh.param<int>("min_keyframe_num", min_frame_num, 20);
        nh.param<float>("force_freq", force_freq, 10.0f);
        nh.param<float>("max_accept_cost", acpt_cost, 0.4f);
        nh.param<int>("thread_num", thread_num, 1);

        nh.param<std::string>("swarm_nodes_config", swarm_node_config, "/home/xuhao/swarm_ws/src/swarm_pkgs/swarm_localization/config/swarm_nodes_test.yaml");

        load_nodes_from_file(swarm_node_config);

        swarm_localization_solver = new SwarmLocalizationSolver(frame_num, min_frame_num, acpt_cost, thread_num);

        nh.param<double>("initial_random_noise", swarm_localization_solver->initial_random_noise, 1.0);

        fused_drone_data_pub = nh.advertise<swarm_msgs::swarm_fused>("/swarm_drones/swarm_drone_fused", 10);
        fused_drone_rel_data_pub = nh.advertise<swarm_msgs::swarm_fused_relative>(
                "/swarm_drones/swarm_drone_fused_relative", 10);
        solving_cost_pub = nh.advertise<std_msgs::Float32>("/swarm_drones/solving_cost", 10);


        ROS_INFO("Will use %d number of keyframe\n", frame_num);
    }
};


int main(int argc, char **argv) {

    ROS_INFO("SWARM VO FUSE ROS\nIniting\n");

    //Use time as seed
    srand(time(NULL));

    ros::init(argc, argv, "swarm_localization");

    ros::NodeHandle nh("swarm_localization");

    SwarmLocalizationNode uwbfusernode(nh);

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();

    return 0;
}
