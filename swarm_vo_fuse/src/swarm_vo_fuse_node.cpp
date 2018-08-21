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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <nav_msgs/Odometry.h>

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


        if (remote_ids_arr.size() == 0)
        {
            //This is first time of receive data
            this->self_id = self_id;
            uwbfuse->self_id = self_id;
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


        //Exange self id and zero
        for(int i = 0; i < drone_num_now; i++)
        {
            auto odom = _self_odoms[i];

            auto position = odom.pose.pose.position;
            auto vel = odom.twist.twist.linear;

            self_pos[i] = Eigen::Vector3d(position.x, position.y, position.z);
            self_vel[i] = Eigen::Vector3d(vel.x, vel.y, vel.z);

            for (int j = 0;j<drone_num_now; j++)
            {
                dis_mat(i,j) = diss[i*drone_num_now + j];
            }
        }

        uwbfuse->id_to_index = ids_index_in_arr;

        uwbfuse->add_new_data_tick(dis_mat, self_pos, self_vel, ids);
        this->uwbfuse->solve();

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

    std::thread solve_thread;
public:
    ros::NodeHandle & nh;
    UWBFuserNode(ros::NodeHandle & _nh):
        nh(_nh)
    {
        recv_remote_drones = nh.subscribe("/swarm_drones/swarm_drone_source_data",1,&UWBFuserNode::on_remote_drones_poses_recieved, this);
        int frame_num = 0, thread_num , min_frame_num;
        nh.param<int>("max_keyframe_num", frame_num, 20);
        nh.param<int>("min_keyframe_num", min_frame_num, 10);

        nh.param<int>("thread_num", thread_num, 4);
        uwbfuse = new UWBVOFuser(frame_num, min_frame_num, thread_num);
       
        fused_drone_data_pub = nh.advertise<swarm_fused>("/swarm_drones/swarm_drone_fused", 1);
        auto cb = new std::function<void(const ID2Vector3d & id2vec, const ID2Vector3d & id2vel)>
        ([&](const ID2Vector3d & id2vec, const ID2Vector3d & id2vel) {
            swarm_fused fused;

            // printf("Pubing !\n");
            for (auto it : id2vec)
            {
                geometry_msgs::Point p;
                geometry_msgs::Vector3 v;
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

                Odometry odom;
                odom.header.frame_id = frame_id;
                odom.pose.pose.position = p;
                odom.twist.twist.linear = v;
                
                pub_odom_id(it.first, odom);
            }

            fused_drone_data_pub.publish(fused);
            return;
        });

        uwbfuse->callback = cb;
        ROS_INFO("Will use %d number of keyframe\n", frame_num);
        uwbfuse->max_frame_number = frame_num;
        uwbfuse->id_to_index = ids_index_in_arr;
        /*
        solve_thread = std::thread([&]{
            while (true)
            {
                this->uwbfuse.solve();
                // usleep(1000000);
            }
        });
        */
    }
private:

    ros::Subscriber recv_remote_drones;
    ros::Publisher fused_drone_data_pub;

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