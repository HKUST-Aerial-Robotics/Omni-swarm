#include "ros/ros.h"
#include <swarm_msgs/swarm_drone_source_data.h>
#include <nav_msgs/Odometry.h>
#include <mavlink/swarm/mavlink.h>
#include <swarm_msgs/remote_uwb_info.h>
#include <swarm_msgs/data_buffer.h>
#include <map>
#include <eigen3/Eigen/Dense>
#include <cstdint>

using namespace swarm_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;

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
    Eigen::Quaterniond quat;



    bool odometry_available = false;
    bool odometry_updated = false;

    nav_msgs::Odometry self_odom;


    Odometry naive_predict(const Odometry & odom_now, double now)
    {
        Odometry ret = odom_now;
        // double now = ros::Time::now().toSec();
        double t_odom = odom_now.header.stamp.toSec();
        // ROS_INFO("Naive predict now %f t_odom %f dt %f", now, t_odom, now-t_odom);
        ret.pose.pose.position.x += (now - t_odom) * odom_now.twist.twist.linear.x;
        ret.pose.pose.position.y += (now - t_odom) * odom_now.twist.twist.linear.y;
        ret.pose.pose.position.z += (now - t_odom) * odom_now.twist.twist.linear.z;
        return ret;
    }

    void on_local_odometry_recv(const nav_msgs::Odometry & odom)
    {

        // ROS_INFO("Odom recv");
        pos.x() = odom.pose.pose.position.x;
        pos.y() = odom.pose.pose.position.y;
        pos.z() = odom.pose.pose.position.z;

        vel.x() = odom.twist.twist.linear.x;
        vel.y() = odom.twist.twist.linear.y;
        vel.z() = odom.twist.twist.linear.z;


        quat.w() = odom.pose.pose.orientation.w;
        quat.x() = odom.pose.pose.orientation.w;
        quat.y() = odom.pose.pose.orientation.w;
        quat.z() = odom.pose.pose.orientation.w;

        self_odom = odom;

        odometry_available = true;
        odometry_updated = true;
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

                odom.pose.pose.orientation.w = swarm_info.q0;
                odom.pose.pose.orientation.x = swarm_info.q1;
                odom.pose.pose.orientation.y = swarm_info.q2;
                odom.pose.pose.orientation.z = swarm_info.q3;

                for (int i = 0; i < 10; i++)
                {
                    _dis[i] = swarm_info.remote_distance[i];
                    // ROS_INFO("id %d %f", i, _dis[i]);
                }              

                return true;  
            }
        }
        return false;
    }

    void send_swarm_mavlink(const float * dis) 
    {

        // ROS_INFO("SENDING MAVLINK");

        mavlink_message_t msg;

        mavlink_msg_swarm_info_pack(0, 0, &msg, odometry_available, pos.x(), pos.y(), pos.z(),
            quat.w(), quat.x(), quat.y(), quat.z(),
            vel.x(), vel.y(), vel.z(), dis);
        int len = mavlink_msg_to_send_buffer(buf , &msg);
        data_buffer buffer;
        buffer.data = std::vector<uint8_t>(buf, buf+len);

        // ROS_INFO("length %d rate %d", len, len*400);

        uwb_senddata_pub.publish(buffer);
    }
    
    std::vector<float> past_self_dis;

    void on_remote_nodes_data_recv(const remote_uwb_info & info)
    {

        int drone_num = info.node_ids.size() + 1;
        const std::vector<unsigned int> & ids = info.node_ids;

        std::vector<std::vector<float>> id_n_distance(100);
        std::map<int, Odometry> id_odoms;


        std::vector<unsigned int> available_id;

        available_id.push_back(info.self_id);

        float self_dis[100] = {0};
        std::vector<float> self_dis_vec(10);

        for (int i = 0; i < 10; i++)
        {
            self_dis[i] = -1;
        }

        for (int i = 0; i < info.node_ids.size(); i++)        
        {
            int _id = info.node_ids[i];

            self_dis[_id] = info.node_dis[i];
            self_dis_vec[_id] = info.node_dis[i];

            // ROS_INFO("i %d id %d %f", i ,_id, self_dis[_id]);

            auto s = info.datas[i];
            nav_msgs::Odometry odom;
            std::vector<float> _dis(10);
            bool ret = parse_mavlink_data(s.data, odom, _dis);
            if (ret)
            {
                id_n_distance[_id] = _dis;
                available_id.push_back(_id);
                id_odoms[_id] = odom;
            }
        }

        if (past_self_dis.size() > 0 && odometry_available)
        {
            id_n_distance[info.self_id] = past_self_dis;
            past_self_dis = self_dis_vec;
        }
        else{
            past_self_dis = self_dis_vec;
            send_swarm_mavlink(self_dis);
            return;
        }

        //Using last distances, assume cost 0.02 time offset
        id_odoms[info.self_id] = naive_predict(self_odom, info.header.stamp.toSec() - 0.02);

        std::vector<float> distance_measure(available_id.size() * available_id.size());

        swarm_drone_source_data data;
        data.ids = available_id;
        data.drone_self_odoms = std::vector<Odometry>(available_id.size());
        data.drone_num = available_id.size();
        data.self_id = info.self_id;
        data.self_frame_id = self_odom.header.frame_id;
        data.header.stamp = info.header.stamp;

        for (int i = 0; i < available_id.size(); i++)
        {

            int _idx = available_id[i];
            data.drone_self_odoms[i] = id_odoms[_idx]; 
            for (int j = 0; j < available_id.size(); j++)
            {
                int _idy = available_id[j];

                // ROS_INFO("idx %d idy %d dis %f", _idx, _idy, id_n_distance[_idx][_idy]);
                if (_idx != _idy)
                    distance_measure[i * available_id.size() + j] = id_n_distance[_idx][_idy];
                else
                    distance_measure[i * available_id.size() + j] = 0;
            }
        }

        send_swarm_mavlink(self_dis);

        data.distance_matrix = distance_measure;
        if (odometry_available && odometry_updated)
        {
            odometry_updated = false;
            swarm_sourcedata_pub.publish(data);
        }
    }

public:
    SwarmDroneProxy(ros::NodeHandle & _nh):
        nh(_nh)
    {
        ROS_INFO("Start SWARM Drone Proxy");

        std::string vins_topic ="";
        nh.param<std::string>("vins_topic", vins_topic, "/vins_estimator/odometry");
        // read /vins_estimator/odometry and send to uwb by mavlink
        local_odometry_sub = nh.subscribe(vins_topic, 1, &SwarmDroneProxy::on_local_odometry_recv, this);
        swarm_data_sub = nh.subscribe("/uwb_node/remote_nodes", 1, &SwarmDroneProxy::on_remote_nodes_data_recv, this);
        swarm_sourcedata_pub = nh.advertise<swarm_drone_source_data>("/swarm_drones/swarm_drone_source_data", 1);
        uwb_senddata_pub = nh.advertise<data_buffer>("/uwb_node/send_broadcast_data", 1);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swarm_drone_proxy");
    ros::NodeHandle nh("swarm_drone_proxy");
    new SwarmDroneProxy(nh);
    ros::spin();
}