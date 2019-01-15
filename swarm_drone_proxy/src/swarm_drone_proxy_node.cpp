#include "ros/ros.h"
#include <swarm_msgs/swarm_drone_source_data.h>
#include <nav_msgs/Odometry.h>
#include <mavlink/swarm/mavlink.h>
#include <swarm_msgs/remote_uwb_info.h>
#include <swarm_msgs/data_buffer.h>
#include <swarm_msgs/swarm_fused.h>
#include <swarm_msgs/swarm_fused_relative.h>
#include <swarm_msgs/swarm_remote_command.h>
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
    ros::Subscriber swarm_rel_sub;
    ros::Subscriber swarm_cmd_sub;
    ros::Publisher swarm_sourcedata_pub;
    ros::Publisher uwb_senddata_pub;
    ros::Publisher drone_cmd_pub;

    uint8_t buf[10000] = {0};

    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond quat;



    bool odometry_available = false;
    bool odometry_updated = false;

    nav_msgs::Odometry self_odom;

    int force_self_id = -1;
    int self_id = -1;
    bool gcs_mode = false;


    Odometry naive_predict(const Odometry & odom_now, double now)
    {
        Odometry ret = odom_now;
        // double now = ros::Time::now().toSec();
        double t_odom = odom_now.header.stamp.toSec();
        // ROS_INFO("Naive predict now %f t_odom %f dt %f", now, t_odom, now-t_odom);
        ret.pose.pose.position.x += (now - t_odom) * odom_now.twist.twist.linear.x;
        ret.pose.pose.position.y += (now - t_odom) * odom_now.twist.twist.linear.y;
        ret.pose.pose.position.z += (now - t_odom) * odom_now.twist.twist.linear.z;
        ret.header.stamp = ros::Time::now();
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
        quat.x() = odom.pose.pose.orientation.x;
        quat.y() = odom.pose.pose.orientation.y;
        quat.z() = odom.pose.pose.orientation.z;

        self_odom = odom;

        odometry_available = true;
        odometry_updated = true;
    }

    bool on_swarm_info_mavlink_msg_recv(mavlink_message_t&  msg, nav_msgs::Odometry & odom, std::vector<float>& _dis) {
        mavlink_swarm_info_t swarm_info;
        
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

    }

    bool parse_mavlink_data(const std::vector<uint8_t> & buf, nav_msgs::Odometry & odom, std::vector<float>& _dis) {
        // mavlink_msg_swa
        mavlink_message_t msg;
        mavlink_status_t status;
        bool ret = false;
        for (uint8_t c : buf) {
            if (mavlink_parse_char(0, c, &msg, &status)) {
                switch(msg.msgid) {
                    case  MAVLINK_MSG_ID_SWARM_INFO:
                        ret = on_swarm_info_mavlink_msg_recv(msg, odom, _dis);
                        break;
                    case MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND: 
                        on_mavlink_recv_swarm_command(msg);
                        break;
                }
            }
        }
        return ret;
    }

    void send_mavlink_message(mavlink_message_t & msg) {
        int len = mavlink_msg_to_send_buffer(buf , &msg);
        data_buffer buffer;
        buffer.data = std::vector<uint8_t>(buf, buf+len);
        uwb_senddata_pub.publish(buffer);
    }

    void send_swarm_mavlink(const float * dis) 
    {

        mavlink_message_t msg;

        mavlink_msg_swarm_info_pack(0, 0, &msg, odometry_available, pos.x(), pos.y(), pos.z(),
            quat.w(), quat.x(), quat.y(), quat.z(),
            vel.x(), vel.y(), vel.z(), dis);

        send_mavlink_message(msg);
    }
    
    std::vector<float> past_self_dis;

    bool force_self_id_avail = false;

    void on_swarm_fused_data_recv(const swarm_fused_relative fused)
    {

        uint8_t buf[1000] = {0};

        mavlink_message_t msg;
        printf("Fused data recv\n");
        if (self_id < 0)
            return;

        for (int i = 0; i < fused.ids.size(); i++)
        {
            uint8_t _id = fused.ids[i];
            if (_id != self_id)
            {
                printf("Send %d to %d rel\n", self_id, _id);
                mavlink_msg_swarm_relative_fused_pack(0, 0, &msg, self_id, _id,
                    fused.relative_drone_position[i].x,
                    fused.relative_drone_position[i].y,
                    fused.relative_drone_position[i].z,
                    fused.relative_drone_yaw[i]);
                send_mavlink_message(msg);
            }
        }
    }

    void on_remote_nodes_data_recv(const remote_uwb_info & info)
    {
        
        int drone_num = info.node_ids.size() + 1;
        const std::vector<unsigned int> & ids = info.node_ids;

        std::vector<std::vector<float>> id_n_distance(100);
        std::map<int, Odometry> id_odoms;


        std::vector<unsigned int> available_id;

        available_id.push_back(info.self_id);
        self_id = info.self_id;

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
                // printf("%d id selfidforce %d\n", _id, force_self_id);
                if (_id == force_self_id)
                    force_self_id_avail = true;
            }
            else {
                if (gcs_mode)
                    ROS_INFO("Drone %d Mavlink Parse Failed", _id);
            }
        }

        if (past_self_dis.size() > 0 && odometry_available)
        {
            id_n_distance[info.self_id] = past_self_dis;
            past_self_dis = self_dis_vec;
        }
        else{
            past_self_dis = self_dis_vec;
            if (!gcs_mode)
                send_swarm_mavlink(self_dis);
            return;
        }

        //Using last distances, assume cost 0.02 time offset
        if (!gcs_mode)
        {
            id_odoms[info.self_id] = naive_predict(self_odom, info.header.stamp.toSec() - 0.02);
        }
        else {
            self_odom.header.stamp = ros::Time::now();
            id_odoms[info.self_id] = self_odom;
        }

        std::vector<float> distance_measure(available_id.size() * available_id.size());

        swarm_drone_source_data data;
        data.ids = available_id;
        data.drone_self_odoms = std::vector<Odometry>(available_id.size());
        data.drone_num = available_id.size();

        if (force_self_id >= 0)
        {
            data.self_id = force_self_id;
        }
        else
        {
            data.self_id = info.self_id;
        }

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

        if (!gcs_mode)
            send_swarm_mavlink(self_dis);


        data.distance_matrix = distance_measure;
        if (force_self_id_avail || force_self_id < 0 && (odometry_available && odometry_updated))
        {
            if (!gcs_mode)
                odometry_updated = false;
            swarm_sourcedata_pub.publish(data);
        }
    }

    void on_send_swarm_commands(swarm_msgs::swarm_remote_command rcmd) {
        mavlink_message_t msg;
        swarm_msgs::drone_onboard_command & cmd = rcmd.cmd;
        // mavlink_msg_swarm_info_pack(0, 0, &msg, odometry_available, pos.x(), pos.y(), pos.z(),
        //     quat.w(), quat.x(), quat.y(), quat.z(),
        //     vel.x(), vel.y(), vel.z(), dis);
        mavlink_msg_swarm_remote_command_pack(0, 0, &msg, rcmd.target_id, cmd.command_type, 
            cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6, cmd.param7, cmd.param8);

        ROS_INFO("Swarm CMD target %d type %d", rcmd.target_id, rcmd.cmd.command_type);
        send_mavlink_message(msg);
    }

    void on_mavlink_recv_swarm_command(mavlink_message_t & msg) {
        mavlink_swarm_remote_command_t cmd;
        mavlink_msg_swarm_remote_command_decode(&msg, &cmd);
        ROS_INFO("Recv swarm remote command to %d", cmd.target_id);        
        if (cmd.target_id == -1 || cmd.target_id == this->self_id) {
            //Cmd apply to this
            drone_onboard_command dcmd;
            dcmd.command_type = cmd.command_type;
            dcmd.param1 = cmd.param1;
            dcmd.param2 = cmd.param2;
            dcmd.param3 = cmd.param3;
            dcmd.param4 = cmd.param4;
            dcmd.param5 = cmd.param5;
            dcmd.param6 = cmd.param6;
            dcmd.param7 = cmd.param7;
            dcmd.param8 = cmd.param8;

            ROS_INFO("Station command target %d type %d", cmd.target_id, dcmd.command_type);
            ROS_INFO("Param 1-4 %d %d %d %d 5-8 %d %d %d %d", 
                dcmd.param1, dcmd.param2, dcmd.param3, dcmd.param4,
                dcmd.param5, dcmd.param6, dcmd.param7, dcmd.param8);
            drone_cmd_pub.publish(dcmd);
        } else {
            return;
        }
    }
public:
    SwarmDroneProxy(ros::NodeHandle & _nh):
        nh(_nh)
    {
        ROS_INFO("Start SWARM Drone Proxy");

        std::string vins_topic ="";
        nh.param<std::string>("vins_topic", vins_topic, "/vins_estimator/odometry");
        nh.param<int>("force_self_id", force_self_id, -1);
        nh.param<bool>("gcs_mode", gcs_mode, false);
        // read /vins_estimator/odometry and send to uwb by mavlink
        local_odometry_sub = nh.subscribe(vins_topic, 1, &SwarmDroneProxy::on_local_odometry_recv, this);
        swarm_data_sub = nh.subscribe("/uwb_node/remote_nodes", 1, &SwarmDroneProxy::on_remote_nodes_data_recv, this);
        swarm_rel_sub = nh.subscribe("/swarm_drones/swarm_drone_fused_relative", 1, &SwarmDroneProxy::on_swarm_fused_data_recv, this);
        swarm_sourcedata_pub = nh.advertise<swarm_drone_source_data>("/swarm_drones/swarm_drone_source_data", 1);
        uwb_senddata_pub = nh.advertise<data_buffer>("/uwb_node/send_broadcast_data", 1);
        swarm_cmd_sub = nh.subscribe("/swarm_drones/send_swarm_command", 1, &SwarmDroneProxy::on_send_swarm_commands, this);
        drone_cmd_pub = nh.advertise<drone_onboard_command>("/drone_commander/onboard_command", 1);
        if (gcs_mode) {
            pos.x() = 0;
            pos.y() = 0;
            pos.z() = 0;
    
            vel.x() = 0;
            vel.y() = 0;
            vel.z() = 0;

            quat.w() = 1;
            quat.x() = 0;
            quat.y() = 0;
            quat.z() = 0;

            self_odom.pose.pose.position.x = 0;
            self_odom.pose.pose.position.y = 0;
            self_odom.pose.pose.position.z = 0;

            self_odom.pose.pose.orientation.w = 0;
            self_odom.pose.pose.orientation.x = 0;
            self_odom.pose.pose.orientation.y = 0;
            self_odom.pose.pose.orientation.z = 0;

            odometry_available = true;
            odometry_updated = true;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swarm_drone_proxy");
    ros::NodeHandle nh("swarm_drone_proxy");
    new SwarmDroneProxy(nh);
    ros::spin();
}