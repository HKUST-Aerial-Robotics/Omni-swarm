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
#include "swarm_localization/swarm_localization.hpp"
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


    corner_array CA_from_msg(const std::vector<swarm_msgs::armarker_detected> &cam0_markers) {
        corner_array ca;
        for (const swarm_msgs::armarker_detected &ad : cam0_markers) {
            for (const swarm_msgs::armarker_corner &_ac : ad.corner_detected) {
                MarkerCornerObservsed mco;

                mco.corner_no = _ac.corner_id;

                mco.observed_point.x() = _ac.x;
                mco.observed_point.y() = _ac.y;
                mco.marker = all_ar_markers[_ac.marker_id];

                ca.push_back(mco);
            }
        }
        return ca;
    }

    NodeFrame node_frame_from_msg(const swarm_msgs::node_frame &_nf) {

        //TODO: Deal with global pose
        if (all_node_defs.find(_nf.id) == all_node_defs.end()) {
            ROS_ERROR("No such node %d", _nf.id);
            exit(-1);
        }
        NodeFrame nf(all_node_defs[_nf.id]);
        nf.stamp = _nf.header.stamp;
        nf.ts = nf.stamp.toNSec();
        nf.frame_available = true;
        nf.vo_available = _nf.vo_available;
        nf.dists_available = !_nf.dismap_ids.empty();

        assert(_nf.dismap_ids.size() == _nf.dismap_dists.size() && "Dismap ids and distance must equal size");

        for (unsigned int i = 0; i < _nf.dismap_ids.size(); i++) {
            nf.dis_map[_nf.dismap_ids[i]] = _nf.dismap_dists[i];
        }

        nf.corner_by_cams.push_back(CA_from_msg(_nf.cam0_markers));
        nf.corner_by_cams.push_back(CA_from_msg(_nf.cam1_markers));

        if (nf.vo_available) {
            nf.self_pose.position.x() = _nf.odometry.pose.pose.position.x;
            nf.self_pose.position.y() = _nf.odometry.pose.pose.position.y;
            nf.self_pose.position.z() = _nf.odometry.pose.pose.position.z;

            nf.self_pose.attitude.w() = _nf.odometry.pose.pose.orientation.w;
            nf.self_pose.attitude.x() = _nf.odometry.pose.pose.orientation.x;
            nf.self_pose.attitude.y() = _nf.odometry.pose.pose.orientation.y;
            nf.self_pose.attitude.z() = _nf.odometry.pose.pose.orientation.z;

            nf.self_vel.x() = _nf.odometry.twist.twist.linear.x;
            nf.self_vel.y() = _nf.odometry.twist.twist.linear.y;
            nf.self_vel.z() = _nf.odometry.twist.twist.linear.z;
        } else {
            if (nf.node->HasVO()) {
                ROS_WARN("Node %d invalid: No vo now", _nf.id);
            }
            nf.is_valid = false;
        }

        return nf;
    }

    SwarmFrame swarm_frame_from_msg(const swarm_msgs::swarm_frame &_sf) {
        SwarmFrame sf;

        sf.stamp = _sf.header.stamp;
        sf.ts = sf.stamp.toNSec();

        for (const swarm_msgs::node_frame &_nf: _sf.node_frames) {
            sf.node_id_list.push_back(_nf.id);
        }

        for (auto it : sf.id2nodeframe) {
            for (auto it_d :it.second.detected_nodes) {
                sf.id2nodeframe[it_d.first].has_detect_relpose = true;
                sf.id2nodeframe[it.first].has_detect_relpose = true;
            }
        }
        return sf;
    }

    double t_last = 0;
protected:
    void on_swarmframe_recv(const swarm_msgs::swarm_frame &_sf) {
        //TODO:Fix when nodeframe invaild!!!!
        ROS_INFO("Recv swarm frame with %ld nodes", _sf.node_frames.size());
        SwarmFrame sf = swarm_frame_from_msg(_sf);


        int _self_id = _sf.self_id;
        frame_id = "world";

        uwbfuse->self_id = _self_id;

        if (remote_ids_arr.empty()) {
            //This is first time of receive data
            this->self_id = _self_id;
            uwbfuse->self_id = self_id;
            ROS_INFO("self id %d", self_id);
            add_drone_id(self_id);
        }

        for (int _id: sf.node_id_list) {
            if (!has_this_drone(_id))
                add_drone_id(_id);
        }


        for (const swarm_msgs::node_frame &_nf: _sf.node_frames) {
            sf.id2nodeframe[_nf.id] = node_frame_from_msg(_nf);
            sf.dis_mat[_nf.id] = sf.id2nodeframe[_nf.id].dis_map;
        }

//        uwbfuse->id_to_index = ids_index_in_arr;
        uwbfuse->all_nodes = remote_ids_arr;

        double t_now = _sf.header.stamp.toSec();

        // printf("Tnow %f\n", t_now);

        if (t_now - t_last > 1 / force_freq) {
            uwbfuse->add_new_swarm_frame(sf);
            std_msgs::Float32 cost;
            cost.data = this->uwbfuse->solve();
            t_last = t_now;
            if (cost.data > 0)
                solving_cost_pub.publish(cost);
        }


    }

    void pub_odom_id(unsigned int id, const Odometry &odom) {
        if (remote_drone_odom_pubs.find(id) == remote_drone_odom_pubs.end()) {
            char name[100] = {0};
            sprintf(name, "/swarm_drones/drone_%d_odom", id);
            remote_drone_odom_pubs[id] = nh.advertise<Odometry>(name, 1);
        }

        auto pub = remote_drone_odom_pubs[id];

        pub.publish(odom);
    }

    nav_msgs::Odometry odom_now;

    void on_drone_odom_recv(const nav_msgs::Odometry &odom) {
        odom_now = odom;
    }

    float force_freq = 10;
    ros::NodeHandle &nh;

private:

    ros::Subscriber recv_remote_drones;
    ros::Subscriber recv_drone_odom_now;
    ros::Publisher fused_drone_data_pub, solving_cost_pub, fused_drone_rel_data_pub;

    std::string frame_id = "";

    std::map<int, ros::Publisher> remote_drone_odom_pubs;
    SwarmLocalizationSolver *uwbfuse = nullptr;

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

    void PredictSwarm(const swarm_frame &_sf) {
        SwarmFrame sf = swarm_frame_from_msg(_sf);
        SwarmFrameState sfs;
        if (this->uwbfuse->PredictSwarm(sf, sfs)) {


        }

    }

public:
    SwarmLocalizationNode(ros::NodeHandle &_nh) :
            nh(_nh) {
        recv_remote_drones = nh.subscribe("/swarm_drones/swarm_frame", 1,
                                          &SwarmLocalizationNode::on_swarmframe_recv, this,
                                          ros::TransportHints().tcpNoDelay());
        recv_drone_odom_now = nh.subscribe("/vins_estimator/imu_propagate", 1,
                                           &SwarmLocalizationNode::on_drone_odom_recv, this,
                                           ros::TransportHints().tcpNoDelay());
        int frame_num = 0, thread_num, min_frame_num;
        float acpt_cost = 0.4;

        std::string swarm_node_config;


        nh.param<int>("max_keyframe_num", frame_num, 100);
        nh.param<int>("min_keyframe_num", min_frame_num, 10);
        nh.param<float>("force_freq", force_freq, 1);
        nh.param<float>("max_accept_cost", acpt_cost, 0.4);
        nh.param<int>("thread_num", thread_num, 4);

        nh.param<std::string>("swarm_nodes_config", swarm_node_config, "/home/xuhao/swarm_ws/src/swarm_pkgs/swarm_localization/config/swarm_nodes_test.yaml");

        load_nodes_from_file(swarm_node_config);

        uwbfuse = new SwarmLocalizationSolver(frame_num, min_frame_num, acpt_cost, thread_num);

        fused_drone_data_pub = nh.advertise<swarm_msgs::swarm_fused>("/swarm_drones/swarm_drone_fused", 10);
        fused_drone_rel_data_pub = nh.advertise<swarm_msgs::swarm_fused_relative>(
                "/swarm_drones/swarm_drone_fused_relative", 10);
        solving_cost_pub = nh.advertise<std_msgs::Float32>("/swarm_drones/solving_cost", 10);


        ROS_INFO("Will use %d number of keyframe\n", frame_num);
        uwbfuse->max_frame_number = frame_num;
//        uwbfuse->id_to_index = ids_index_in_arr;
    }
};


int main(int argc, char **argv) {

    ROS_INFO("SWARM VO FUSE ROS\nIniting\n");

    //Use time as seed
    srand(time(NULL));

    ros::init(argc, argv, "swarm_localization");

    ros::NodeHandle nh("swarm_localization");

    SwarmLocalizationNode uwbfusernode(nh);

    ros::spin();

    return 0;
}


/*
            Odometry odom_now = sfs.node_poses[self_id];

            swarm_msgs::swarm_fused fused;
            swarm_msgs::swarm_fused_relative relative_fused;


            ros::Time ts_now = odom_now.header.stamp;
            double dt = (ts_now - ts).toSec();

            fused.header.stamp = ts_now;
            relative_fused.header.stamp = ts_now;

            // ROS_INFO("tnow %f, ts %f, dt %f", ts_now.toSec(), ts.toSec(), dt);

            Eigen::Vector3d self_pos;
            Eigen::Vector3d self_vel;
            Eigen::Quaterniond self_quat;

            self_pos.x() = odom_now.pose.pose.position.x;
            self_pos.y() = odom_now.pose.pose.position.y;
            self_pos.z() = odom_now.pose.pose.position.z;

            self_vel.x() = odom_now.twist.twist.linear.x;
            self_vel.y() = odom_now.twist.twist.linear.y;
            self_vel.z() = odom_now.twist.twist.linear.z;

            self_quat.w() = odom_now.pose.pose.orientation.w;
            self_quat.x() = odom_now.pose.pose.orientation.x;
            self_quat.y() = odom_now.pose.pose.orientation.y;
            self_quat.z() = odom_now.pose.pose.orientation.z;
            // printf("Pubing !\n");
            for (auto it : id2vec) {
                Quaterniond quat = id2quat.at(it.first);

                geometry_msgs::Point p, rel_p;
                geometry_msgs::Vector3 v, rel_v;
                geometry_msgs::Quaternion q;

                if (it.first == this->self_id) {
                    p.x = self_pos.x();
                    p.y = self_pos.y();
                    p.z = self_pos.z();

                    v.x = self_vel.x();
                    v.y = self_vel.y();
                    v.z = self_vel.z();

                    rel_p.x = 0;
                    rel_p.y = 0;
                    rel_p.z = 0;

                    rel_v.x = 0;
                    rel_v.y = 0;
                    rel_v.z = 0;

                    q = odom_now.pose.pose.orientation;
                } else {

                    p.x = it.second.x() + id2vel.at(it.first).x() * dt;
                    p.y = it.second.y() + id2vel.at(it.first).y() * dt;
                    p.z = it.second.z() + id2vel.at(it.first).z() * dt;

                    //May cause error
                    v.x = id2vel.at(it.first).x();
                    v.y = id2vel.at(it.first).y();
                    v.z = id2vel.at(it.first).z();


                    rel_p.x = it.second.x() - self_pos.x();
                    rel_p.y = it.second.y() - self_pos.y();
                    rel_p.z = it.second.z() - self_pos.z();
                    rel_v.x = id2vel.at(it.first).x() - self_vel.x();
                    rel_v.y = id2vel.at(it.first).y() - self_vel.y();
                    rel_v.z = id2vel.at(it.first).z() - self_vel.z();

                    q.w = quat.w();
                    q.x = quat.x();
                    q.y = quat.y();
                    q.z = quat.z();
                }

                fused.ids.push_back(it.first);
                fused.remote_drone_position.push_back(p);
                fused.remote_drone_velocity.push_back(v);
                fused.remote_drone_attitude.push_back(q);

                Eigen::Quaterniond rel_quat = self_quat.inverse() * quat;
                Vector3d euler = rel_quat.toRotationMatrix().eulerAngles(0, 1, 2);
                double rel_yaw = euler.z();

                relative_fused.ids.push_back(it.first);
                relative_fused.relative_drone_position.push_back(rel_p);
                relative_fused.relative_drone_velocity.push_back(rel_v);
                relative_fused.relative_drone_yaw.push_back(rel_yaw);

                Odometry odom;
                odom.header.frame_id = frame_id;
                odom.header.stamp = odom_now.header.stamp;
                odom.pose.pose.position = p;

                odom.pose.pose.orientation = q;
                odom.twist.twist.linear = v;
                if (it.first != self_id)
                    pub_odom_id(it.first, odom);
                else
                    pub_odom_id(it.first, odom_now);
            }

            fused_drone_data_pub.publish(fused);
            fused_drone_rel_data_pub.publish(relative_fused);*/