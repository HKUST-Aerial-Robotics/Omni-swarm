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
#include <chrono>
#include <swarm_msgs/swarm_drone_basecoor.h>
#include <swarm_msgs/LoopConnection.h>

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
    backward::SignalHandling sh;
}

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace Eigen;
using namespace nav_msgs;
using namespace swarm_msgs;
using namespace std::chrono;



Eigen::Matrix<double, 2, 3> tangent_base_for_unit_detect(const Eigen::Vector3d & pts_j) {
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

class SwarmLocalizationNode {

    void add_drone_id(int _id) {
        this->remote_ids_arr.push_back(_id);
        this->remote_ids_set.insert(_id);
        this->ids_index_in_arr[_id] = this->remote_ids_arr.size() - 1;
    }

    bool has_this_drone(int _id) {
        return remote_ids_set.find(_id) != remote_ids_set.end();
    }


    bool nodedef_has_id(int _id) const {
        return all_node_defs.find(_id) != all_node_defs.end();
    }

    NodeFrame node_frame_from_msg(const swarm_msgs::node_frame &_nf) const {

        //TODO: Deal with global pose
        if (!nodedef_has_id(_nf.id)) {
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
            if (nodedef_has_id(_nf.dismap_ids[i])) {
                // nf.dis_map[_nf.dismap_ids[i]] = _nf.dismap_dists[i] + nf.bias(_nf.dismap_ids[i]);
                nf.dis_map[_nf.dismap_ids[i]] = nf.to_real_distance(_nf.dismap_dists[i], _nf.dismap_ids[i]);
            }

        }

        if (nf.vo_available) {
            nf.self_pose = Pose(_nf.position, _nf.yaw);
            // ROS_WARN("Node %d vo valid", _nf.id);
            nf.is_valid = true;

        } else {
            if (nf.node->has_odometry()) {
                // ROS_WARN_THROTTLE(1.0, "Node %d invalid: No vo now", _nf.id);
                // ROS_WARN("Node %d invalid: No vo now", _nf.id);
            }
            nf.is_valid = false;
        }

        for (auto nd: _nf.detected) {
            if (nodedef_has_id(nd.remote_drone_id)) {
                nf.detected_nodes[nd.remote_drone_id] = Pose(nd.dpos, nd.dyaw);
                if(nd.is_2d_detect) {
                    nf.detected_nodes[nd.remote_drone_id].pos() = nf.detected_nodes[nd.remote_drone_id].pos().normalized();
                    nf.detect_tan_base[nd.remote_drone_id] = tangent_base_for_unit_detect(nf.detected_nodes[nd.remote_drone_id].pos());
                }
                nf.detected_nodes_posvar[nd.remote_drone_id] = 
                    Eigen::Vector3d(nd.dpos_cov.x, nd.dpos_cov.y, nd.dpos_cov.z);
                nf.has_detect_relpose = true;
            }
        }
        return nf;
    }

    SwarmFrame swarm_frame_from_msg(const swarm_msgs::swarm_frame &_sf) const {
        SwarmFrame sf;

        sf.stamp = _sf.header.stamp;
        sf.ts = sf.stamp.toNSec();
        sf.self_id = _sf.self_id;

        for (const swarm_msgs::node_frame &_nf: _sf.node_frames) {
            if (nodedef_has_id(_nf.id)) {
                NodeFrame nf = node_frame_from_msg(_nf);
                //Set nf ts to sf ts here; Trick for early version
                nf.ts = sf.ts;

                if (nf.is_static || (!nf.is_static && nf.vo_available)) { //If not static then must has vo
                    sf.id2nodeframe[_nf.id] = nf;
                    sf.node_id_list.insert(_nf.id);
                    sf.dis_mat[_nf.id] = sf.id2nodeframe[_nf.id].dis_map;
                }
            }
        }

        for (auto it : sf.id2nodeframe) {
            for (auto it_d :it.second.detected_nodes) {
                //Add only in node id list
                if ( sf.id2nodeframe.find(it_d.first) != sf.id2nodeframe.end() && 
                    sf.id2nodeframe.find(it_d.first) != sf.id2nodeframe.end() ) {
                    sf.id2nodeframe[it_d.first].has_detect_relpose = true;
                    sf.id2nodeframe[it.first].has_detect_relpose = true;
                    // ROS_INFO("Has detection %d by %d", it_d.first, it.first);
                }
            }
        }

        return sf;
    }


    void on_loop_connection_received(const swarm_msgs::LoopConnection & loop_conn) {
        ROS_INFO("Add new loop connection from %d to %d", loop_conn.id_a, loop_conn.id_b);
        this->swarm_localization_solver->add_new_loop_connection(loop_conn);
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

        double t_now = _sf.header.stamp.toSec();

        swarm_localization_solver->add_new_swarm_frame(sf);
        // printf("Tnow %f DT %f\n", t_now, t_now - t_last);
        // For some bags if (t_now - t_last > 1 / force_freq && (t_now - t_last < 10 || t_last <1e-4)) {
        if (t_now - t_last > 1 / force_freq) {// && (t_now - t_last < 10 || t_last <1e-4)) {
            std_msgs::Float32 cost;
            // ROS_INFO("Try to solve");
            cost.data = this->swarm_localization_solver->solve();
            t_last = t_now;
            if (cost.data > 0)
                solving_cost_pub.publish(cost);
        }
    }

    void pub_posevel_id(unsigned int id, const Pose & pose, const Eigen::Matrix4d cov, const Eigen::Vector3d vel, ros::Time stamp) {
        Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "world";
        odom.pose.pose = pose.to_ros_pose();
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        for (int i = 0; i < 4; i++) {
            for (int j =0; j < 4; j++) {
                int _i = i;
                int _j = j;
                if (_i == 3) {
                    _i = 5;
                }
                if (_j == 3) {
                    _j = 5;
                }
                odom.pose.covariance[_i*6+_j] = cov(i, j);
            }
        }
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
    ros::Subscriber loop_connection_sub;
    ros::Publisher fused_drone_data_pub, fused_drone_basecoor_pub, solving_cost_pub, fused_drone_rel_data_pub;

    std::string frame_id = "";

    std::map<int, ros::Publisher> remote_drone_odom_pubs;
    SwarmLocalizationSolver *swarm_localization_solver = nullptr;

    std::vector<int> remote_ids_arr;
    std::set<int> remote_ids_set;
    std::map<int, int> ids_index_in_arr;
    std::map<int, Node *> all_node_defs;

    ros::Timer timer;

    bool pub_swarm_odom = false;


    int self_id = -1;

    float predict_freq;

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
                    auto ann_pos = new_node->get_anntena_pos();
                    ROS_INFO("NODE %d static:%d vo %d uwb %d ann %5.4f %5.4f %5.4f",
                             new_node->id,
                             new_node->is_static_node(),
                             new_node->has_odometry(),
                             new_node->has_uwb(),
                             ann_pos.x(),
                             ann_pos.y(),
                             ann_pos.z()
                    );
                    
                }

        } catch (std::exception & e) {
            ROS_ERROR("Error while parsing config file:%s, exit",e.what());
            exit(-1);
        }

    }

    void pub_fused_relative(const SwarmFrameState & _sfs, ros::Time stamp) {
        if (_sfs.node_poses.size() <= 1) {
            return;
        } 
        swarm_fused_relative sfr;
        swarm_fused sf;
        swarm_drone_basecoor sdb;

        sf.header.stamp = stamp;
        sfr.header.stamp = stamp;
        sdb.header.stamp = stamp;
        Pose self_pose = _sfs.node_poses.at(self_id);
        for (auto it : _sfs.node_poses) {
            int id = it.first;
            Pose _pose = it.second;
            Pose DPose = Pose::DeltaPose(self_pose, _pose, true);

            double dyaw = DPose.yaw();
            sfr.ids.push_back(id);
            sfr.relative_drone_position.push_back(DPose.to_ros_pose().position);
            sfr.relative_drone_yaw.push_back(dyaw);

            sf.ids.push_back(id);
            sf.local_drone_position.push_back(_pose.to_ros_pose().position);
            sf.local_drone_yaw.push_back(_pose.yaw());
            geometry_msgs::Vector3 pcov;
            pcov.x = _sfs.node_covs.at(id)(0, 0);
            pcov.y = _sfs.node_covs.at(id)(1, 1);
            pcov.z = _sfs.node_covs.at(id)(2, 2);
            sf.position_cov.push_back(pcov);
            sf.yaw_cov.push_back(_sfs.node_covs.at(id)(3,3));

            sfr.position_cov.push_back(pcov);
            sfr.yaw_cov.push_back(_sfs.node_covs.at(id)(3,3));

            //Temp disable veloctiy
            geometry_msgs::Vector3 spd;
            spd.x = 0;
            spd.y = 0;
            spd.z = 0;
            sfr.relative_drone_velocity.push_back(spd);
            sf.local_drone_velocity.push_back(spd);

            sdb.ids.push_back(id);
            Pose _coor = _sfs.base_coor_poses.at(id);
            sdb.drone_basecoor.push_back(_coor.to_ros_pose().position);
            sdb.drone_baseyaw.push_back(_coor.yaw());
            geometry_msgs::Vector3 pcov2;
            pcov2.x = _sfs.base_coor_covs.at(id)(0, 0);
            pcov2.y = _sfs.base_coor_covs.at(id)(1, 1);
            pcov2.z = _sfs.base_coor_covs.at(id)(2, 2);
            sdb.position_cov.push_back(pcov2);
            sdb.yaw_cov.push_back(_sfs.base_coor_covs.at(id)(3,3));
        }

        fused_drone_rel_data_pub.publish(sfr);
        fused_drone_data_pub.publish(sf);
        fused_drone_basecoor_pub.publish(sdb);
    }


    double t_last_predict_swarm = 0;

    void predict_swarm(const swarm_frame &_sf) {
        double t_now = ros::Time::now().toSec();
        if (t_now - t_last_predict_swarm > 1.0/predict_freq) {
            t_last_predict_swarm = t_now;
            high_resolution_clock::time_point t1 = high_resolution_clock::now();
            if (_sf.node_frames.size() >= 1) {
                if (swarm_localization_solver->CanPredictSwarm()) {
                    SwarmFrame sf = swarm_frame_from_msg(_sf);
                    SwarmFrameState _sfs = swarm_localization_solver->PredictSwarm(sf);
                    if (pub_swarm_odom) {
                        for (auto it: _sfs.node_poses) {
                            this->pub_posevel_id(it.first, it.second, _sfs.node_covs[it.first], _sfs.node_vels[it.first], sf.stamp);
                        }
                    }
                    pub_fused_relative(_sfs, sf.stamp);
                } else {
                    ROS_WARN_THROTTLE(1.0, "Unable to predict swarm");
                }
                
                high_resolution_clock::time_point t2 = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>( t2 - t1 ).count();
                // double dts = (ros::Time::now() - ts).toSec();
                //ROS_INFO_THROTTLE(1.0, "Predict cost %ld mus", duration);
            }
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
        
        loop_connection_sub = nh.subscribe("/swarm_loop/loop_connection", 10, 
                                    &SwarmLocalizationNode::on_loop_connection_received, this, 
                                    ros::TransportHints().tcpNoDelay());
        
        std::string swarm_node_config;

        swarm_localization_solver_params solver_params;

        nh.param<int>("max_keyframe_num", solver_params.max_frame_number, 50);
        nh.param<int>("dense_keyframe_num", solver_params.dense_frame_number, 20);
        nh.param<int>("min_keyframe_num", solver_params.min_frame_number, 3);
        nh.param<float>("force_freq", force_freq, 1.0f);
        nh.param<float>("predict_freq", predict_freq, 10.0f);
        nh.param<float>("max_accept_cost", solver_params.acpt_cost, 10.0f);
        nh.param<float>("min_kf_movement", solver_params.kf_movement, 0.4f);
        nh.param<float>("init_xy_movement", solver_params.init_xy_movement, 2.0f);
        nh.param<float>("init_z_movement", solver_params.init_z_movement, 1.0f);
        nh.param<int>("thread_num", solver_params.thread_num, 1);
        nh.param<bool>("pub_swarm_odom", pub_swarm_odom, false);
        nh.param<std::string>("cgraph_path", solver_params.cgraph_path, "/home/dji/cgraph.dot");

        nh.param<std::string>("swarm_nodes_config", swarm_node_config, "/home/xuhao/swarm_ws/src/swarm_pkgs/swarm_localization/config/swarm_nodes5.yaml");

        load_nodes_from_file(swarm_node_config);
        swarm_localization_solver = new SwarmLocalizationSolver(solver_params);
        fused_drone_data_pub = nh.advertise<swarm_msgs::swarm_fused>("/swarm_drones/swarm_drone_fused", 10);
        fused_drone_basecoor_pub = nh.advertise<swarm_msgs::swarm_drone_basecoor>("/swarm_drones/swarm_drone_basecoor", 10);
        fused_drone_rel_data_pub = nh.advertise<swarm_msgs::swarm_fused_relative>(
                "/swarm_drones/swarm_drone_fused_relative", 10);
        solving_cost_pub = nh.advertise<std_msgs::Float32>("/swarm_drones/solving_cost", 10);


        ROS_INFO("Will use %d number of keyframe\n", solver_params.max_frame_number);
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
    //ros::spin();

    return 0;
}
