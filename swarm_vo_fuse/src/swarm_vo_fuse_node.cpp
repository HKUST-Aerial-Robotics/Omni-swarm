#include <iostream>
#include "glog/logging.h"
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include <vector>
#include "ros/ros.h"
#include <swarm_msgs/swarm_drone_source_data.h>
#include <algorithm>
#include <set>
#include <map>
#include <time.h>
#include <thread>  
#include <unistd.h>
#include "swarm_vo_fuse/swarm_vo_fuser.hpp"
#include "swarm_msgs/swarm_fused.h"
#include "swarm_msgs/swarm_fused_relative.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>


using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace swarm_msgs;
using namespace Eigen;
using namespace nav_msgs;



class UWBFuserNode{

    void add_drone_id(int _id)
    {
        this->remote_ids_arr.push_back(_id);
        this->remote_ids_set.insert(_id);
        this->ids_index_in_arr[_id] = this->remote_ids_arr.size() - 1;
    }

    bool has_this_drone(int _id)
    {
        return remote_ids_set.find(_id) != remote_ids_set.end();
    }

    int get_drone_num_now()
    {
        return remote_ids_arr.size();
    }

    double t_last = 0;
protected:
    void on_remote_drones_poses_recieved(swarm_drone_source_data rdp)
    {

        // ROS_INFO("Recv remote drone poses");
        int self_id = rdp.self_id;
        auto ids = rdp.ids;
        auto diss = rdp.distance_matrix;
        auto _self_odoms = rdp.drone_self_odoms;
        int drone_num_now = rdp.drone_num;

        frame_id = rdp.self_frame_id;

        uwbfuse->self_id = self_id;


        if (remote_ids_arr.size() == 0)
        {
            //This is first time of receive data
            this->self_id = self_id;
            uwbfuse->self_id = self_id;
            ROS_INFO("self id %d", self_id);
            add_drone_id(self_id);
        }

        for (int _id:rdp.ids)
        {
            if (!has_this_drone(_id))
                add_drone_id(_id);
        }

        Eigen::MatrixXd dis_mat(drone_num_now, drone_num_now);
        vec_array self_pos(drone_num_now);
        vec_array self_vel(drone_num_now);
        quat_array self_quat(drone_num_now);


        //Exange self id and zero
        for(int i = 0; i < drone_num_now; i++)
        {
            auto odom = _self_odoms[i];

            auto position = odom.pose.pose.position;
            auto quat = odom.pose.pose.orientation;
            auto vel = odom.twist.twist.linear;

            self_pos[i] = Eigen::Vector3d(position.x, position.y, position.z);
            // ROS_INFO("SP %d  %f %f %f", i, self_pos[i].x(), self_pos[i].y(), self_pos[i].z());
            self_vel[i] = Eigen::Vector3d(vel.x, vel.y, vel.z);
            self_quat[i] = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);

            for (int j = 0;j<drone_num_now; j++)
            {
                dis_mat(i,j) = diss[i*drone_num_now + j];
            }
        }

        uwbfuse->id_to_index = ids_index_in_arr;

        double t_now = rdp.header.stamp.toSec();

        // printf("Tnow %f\n", t_now);
        
        if (t_now - t_last > 1 / force_freq)
        {
            uwbfuse->add_new_data_tick(dis_mat, self_pos, self_vel, self_quat, ids, rdp.header.stamp);
            std_msgs::Float32 cost;
            cost.data = this->uwbfuse->solve();
            t_last = t_now;
            if (cost.data > 0)
                solving_cost_pub.publish(cost);
        }


    }

    void pub_odom_id(unsigned int id, const Odometry & odom)
    {
        if (remote_drone_odom_pubs.find(id) == remote_drone_odom_pubs.end())
        {
            char name[100] = {0};
            sprintf(name, "/swarm_drones/drone_%d_odom", id);
            remote_drone_odom_pubs[id] = nh.advertise<Odometry>(name, 1);
        }

        auto pub = remote_drone_odom_pubs[id];

        pub.publish(odom);
    }

    nav_msgs::Odometry odom_now;
    void on_drone_odom_recv(nav_msgs::Odometry odom) {
        odom_now = odom;
    }

    float force_freq = 10;
public:
    ros::NodeHandle & nh;
    UWBFuserNode(ros::NodeHandle & _nh):
        nh(_nh)
    {
        recv_remote_drones = nh.subscribe("/swarm_drones/swarm_drone_source_data",1,&UWBFuserNode::on_remote_drones_poses_recieved, this);
        recv_drone_odom_now = nh.subscribe("/vins_estimator/imu_propagate",1,&UWBFuserNode::on_drone_odom_recv, this);
        int frame_num = 0, thread_num , min_frame_num;
        float acpt_cost = 0.4;  

        float anntenna_pos_x = 0, anntenna_pos_y = 0,anntenna_pos_z = 0.27;

        nh.param<int>("max_keyframe_num", frame_num, 20);
        nh.param<int>("min_keyframe_num", min_frame_num, 10);
        nh.param<float>("force_freq", force_freq, 10);
        nh.param<float>("max_accept_cost", acpt_cost, 0.4);
        nh.param<int>("thread_num", thread_num, 4);

        nh.param<float>("anntenna_pos/x", anntenna_pos_x, 0);
        nh.param<float>("anntenna_pos/y", anntenna_pos_y, 0);
        nh.param<float>("anntenna_pos/z", anntenna_pos_z, 0.27);


        Eigen::Vector3d ann_pos(anntenna_pos_x, anntenna_pos_y, anntenna_pos_z);

        uwbfuse = new UWBVOFuser(frame_num, min_frame_num, ann_pos, acpt_cost, thread_num);
       
        fused_drone_data_pub = nh.advertise<swarm_fused>("/swarm_drones/swarm_drone_fused", 1);
        fused_drone_rel_data_pub = nh.advertise<swarm_fused_relative>("/swarm_drones/swarm_drone_fused_relative", 1);
        solving_cost_pub = nh.advertise<std_msgs::Float32>("/swarm_drones/solving_cost", 10);
        uwbfuse->callback = [&](const ID2Vector3d & id2vec, const ID2Vector3d & id2vel, const ID2Quat & id2quat) {
            swarm_fused fused;
            swarm_fused_relative relative_fused;
            Eigen::Vector3d self_pos = id2vec.at(self_id);
            Eigen::Vector3d self_vel = id2vel.at(self_id);
            Eigen::Quaterniond self_quat = id2quat.at(self_id);

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
            for (auto it : id2vec)
            {
                geometry_msgs::Point p, rel_p;
                geometry_msgs::Vector3 v, rel_v;
                p.x = it.second.x();
                p.y = it.second.y();
                p.z = it.second.z();

                //May cause error
                v.x = id2vel.at(it.first).x();
                v.y = id2vel.at(it.first).y();
                v.z = id2vel.at(it.first).z();

                fused.ids.push_back(it.first);
                fused.remote_drone_position.push_back(p);
                fused.remote_drone_velocity.push_back(v);

                Quaterniond quat = id2quat.at(it.first);
                
                rel_p.x = it.second.x() - self_pos.x();
                rel_p.y = it.second.y() - self_pos.y();
                rel_p.z = it.second.z() - self_pos.z();

                rel_v.x = id2vel.at(it.first).x() - self_vel.x();
                rel_v.y = id2vel.at(it.first).y() - self_vel.y();
                rel_v.z = id2vel.at(it.first).z() - self_vel.z();

                Eigen::Quaterniond rel_quat =  self_quat.inverse() * quat;
                Vector3d euler = rel_quat.toRotationMatrix().eulerAngles(0, 1, 2);
                double rel_yaw = euler.z();

                relative_fused.ids.push_back(it.first);
                relative_fused.relative_drone_position.push_back(rel_p);
                relative_fused.relative_drone_velocity.push_back(rel_v);
                relative_fused.relative_drone_yaw.push_back(rel_yaw);

                Odometry odom;
                odom.header.frame_id = frame_id;
                odom.pose.pose.position = p;
                
                odom.pose.pose.orientation.w = quat.w();
                odom.pose.pose.orientation.x = quat.x();
                odom.pose.pose.orientation.y = quat.y();
                odom.pose.pose.orientation.z = quat.z();

                odom.twist.twist.linear = v;
                
                pub_odom_id(it.first, odom);
            }

            fused_drone_data_pub.publish(fused);
            fused_drone_rel_data_pub.publish(relative_fused);
            return;
        };

        ROS_INFO("Will use %d number of keyframe\n", frame_num);
        uwbfuse->max_frame_number = frame_num;
        uwbfuse->id_to_index = ids_index_in_arr;
    }
private:

    ros::Subscriber recv_remote_drones;
    ros::Subscriber recv_drone_odom_now;
    ros::Publisher fused_drone_data_pub, solving_cost_pub, fused_drone_rel_data_pub;

    std::string frame_id = "";

    std::map<int, ros::Publisher> remote_drone_odom_pubs;
    UWBVOFuser * uwbfuse = nullptr;

    std::vector<int> remote_ids_arr;
    std::set<int> remote_ids_set;
    std::map<int,int> ids_index_in_arr;

    ros::Timer timer;


    int self_id = -1;

};



int main(int argc, char ** argv)
{

    ROS_INFO("SWARM VO FUSE ROS\nIniting\n");

    //Use time as seed
    srand (time(NULL));
    
    ros::init(argc, argv, "swarm_vo_fuse");

    ros::NodeHandle nh("swarm_vo_fuse");

    UWBFuserNode uwbfusernode(nh);

    ros::spin();

    return 0;
}