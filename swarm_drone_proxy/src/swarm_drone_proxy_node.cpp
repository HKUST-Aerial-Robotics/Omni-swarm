#include "ros/ros.h"
#include <swarm_drone_proxy/swarm_drone_source_data.h>
#include <nav_msgs/Odometry.h>
#include <mavlink/swarm/mavlink.h>
#include <infinity_uwb_ros/remote_uwb_info.h>
#include <infinity_uwb_ros/data_buffer.h>
#include <map>
#include <eigen3/Eigen/Dense>
#include <cstdint>

using namespace swarm_drone_proxy;
using namespace infinity_uwb_ros;
using namespace nav_msgs;

class SwarmDroneProxy
{
    ros::NodeHandle & nh;
    
    ros::Subscriber local_odometry_sub;
    ros::Subscriber swarm_data_sub;
    ros::Publisher swarm_sourcedata_pub;
    ros::Publisher uwb_senddata_pub;

    uint8_t buf[10000] = {0};

    Eigen::Vector3d pos;
    Eigen::Vector3d vel;

    bool odometry_available = false;


    void on_local_odometry_recv(const nav_msgs::Odometry & odom)
    {
        pos.x() = odom.pose.pose.position.x;
        pos.y() = odom.pose.pose.position.y;
        pos.z() = odom.pose.pose.position.z;

        vel.x() = odom.twist.twist.linear.x;
        vel.y() = odom.twist.twist.linear.y;
        vel.z() = odom.twist.twist.linear.z;
    }

    bool parse_mavlink_data(const std::vector<uint8_t> & buf, nav_msgs::Odometry & odom, std::vector<float>& _dis)
    {
        // mavlink_msg_swa
        mavlink_message_t msg;
        mavlink_status_t status;
        mavlink_swarm_info_t swarm_info;
        for (uint8_t c : buf)
        {
            if (mavlink_parse_char(0, c, &msg, &status) && msg.msgid == MAVLINK_MSG_ID_SWARM_INFO)
            {
                mavlink_msg_swarm_info_decode(&msg, &swarm_info);

                if (!swarm_info.odom_vaild)
                    return false;
                odom.pose.pose.position.x = swarm_info.x;
                odom.pose.pose.position.y = swarm_info.y;
                odom.pose.pose.position.z = swarm_info.z;
                
                odom.twist.twist.linear.x = swarm_info.vx;
                odom.twist.twist.linear.y = swarm_info.vy;
                odom.twist.twist.linear.z = swarm_info.vz;

                for (int i = 0; i < 10; i++)
                {
                    _dis[i] = swarm_info.remote_distance[i];
                }              

                return true;  
            }
        }
        return false;
    }

    void send_swarm_mavlink(const float * dis) 
    {
        mavlink_message_t msg;

        mavlink_msg_swarm_info_pack(0, 0, &msg, odometry_available, pos.x(), pos.y(), pos.z(), 
            vel.x(), vel.y(), vel.z(), dis);
        int len = mavlink_msg_to_send_buffer(buf , &msg);
        data_buffer buffer;
        buffer.data = std::vector<uint8_t>(buf, buf+len);

        uwb_senddata_pub.publish(buffer);
    }
    
    std::vector<float> past_self_dis;

    void on_remote_nodes_data_recv(const infinity_uwb_ros::remote_uwb_info & info)
    {
        int drone_num = info.node_ids.size();
        const std::vector<unsigned int> & ids = info.node_ids;

        std::vector<std::vector<float>> id_n_distance(100);
        std::vector<Odometry> id_odoms;


        std::vector<unsigned int> available_id;

        available_id.push_back(info.self_id);

        float self_dis[100] = {0};

        for (int i = 0; i < drone_num; i++)
        {
            self_dis[i] = -1;
        }

        for (int i = 0; i < info.node_ids.size(); i++)        
        {
            int _id = info.node_ids[i];

            self_dis[_id] = info.node_dis[i];

            auto s = info.datas[i];
            nav_msgs::Odometry odom;
            std::vector<float> _dis(drone_num);
            bool ret = parse_mavlink_data(s.data, odom, _dis);
            if (ret)
            {
                id_n_distance[_id] = _dis;
                available_id.push_back(_id);
                id_odoms[_id] = odom;
            }
        }

        if (past_self_dis.size() > 0)
        {
            id_n_distance[info.self_id] = past_self_dis;
        }
        else{
            past_self_dis = info.node_dis; 
            send_swarm_mavlink(self_dis);
            return;
        }
        
        past_self_dis = info.node_dis;



        std::vector<float> distance_measure(available_id.size() * available_id.size());

        swarm_drone_source_data data;
        data.ids = available_id;
        data.drone_self_poses = std::vector<Odometry>(available_id.size());
        data.drone_num = available_id.size();
        data.self_id = info.self_id;

        for (int i = 0; i < available_id.size(); i++)
        {
            for (int j = 0; j < available_id.size(); j++)
            {
                int _idx = available_id[i];
                int _idy = available_id[j];

                distance_measure[i * available_id.size() + j] = id_n_distance[_idx][_idy];
            }
        }

        send_swarm_mavlink(self_dis);

        data.distance_matrix = distance_measure;

        swarm_sourcedata_pub.publish(data);
    }

public:
    SwarmDroneProxy(ros::NodeHandle & _nh):
        nh(_nh)
    {
        // read /vins_estimator/odometry and send to uwb by mavlink
        local_odometry_sub = nh.subscribe("/vins_estimator/odometry", 1, &SwarmDroneProxy::on_local_odometry_recv, this);
        swarm_data_sub = nh.subscribe("/uwb_node/remote_nodes", 1, &SwarmDroneProxy::on_remote_nodes_data_recv, this);
        swarm_sourcedata_pub = nh.advertise<swarm_drone_source_data>("/swarm_drones/swarm_drone_source_data", 1);
        uwb_senddata_pub = nh.advertise<data_buffer>("send_broadcast_data", 1);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swarm_drone_proxy");
    ros::NodeHandle nh("swarm_drone_proxy");

    ros::spin();
}