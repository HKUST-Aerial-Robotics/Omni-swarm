#include <iostream>
#include "uwb_helper.h"
#include <unistd.h>
#include <ros/ros.h>
#include "uwb_helper.h"
#include <infinity_uwb_ros/remote_uwb_info.h>
#include <infinity_uwb_ros/incoming_broadcast_data.h>
#include <infinity_uwb_ros/data_buffer.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>


class UWBRosNodeofNode: public UWBHelperNode
{
public:
    UWBRosNodeofNode(std::string serial_name, int baudrate, ros::NodeHandle nh,bool enable_debug):
        UWBHelperNode(serial_name, baudrate, false)

    {
        remote_node_pub = nh.advertise<infinity_uwb_ros::remote_uwb_info>("remote_nodes", 1);
        broadcast_data_pub = nh.advertise<infinity_uwb_ros::incoming_broadcast_data>("incoming_broadcast_data",1);

        recv_bdmsg = nh.subscribe("send_broadcast_data", 1, &UWBRosNodeofNode::on_send_broadcast_req, this);   
    }

protected:

    virtual void on_send_broadcast_req(infinity_uwb_ros::data_buffer msg)
    {
        
        this->send_broadcast_data(msg.data);
    }
    virtual void on_broadcast_data_recv(int _id, int _recv_time, Buffer _msg) override
    {
        UWBHelperNode::on_broadcast_data_recv(_id, _recv_time, _msg);
        // printf("Recv broadcast data %s", (char*)_msg.data());
        
        infinity_uwb_ros::incoming_broadcast_data data;
        data.header.stamp = ros::Time::now();
        data.lps_time = sys_time;
        data.remote_id = _id;
        data.remote_recv_time = _recv_time;
        std::string ret((char*)_msg.data(), _msg.size());
        data.data = ret;
        broadcast_data_pub.publish(data);
    }

    virtual void on_node_data_updated() override
    {
        UWBHelperNode::on_node_data_updated();
     
        static int count = 0;
        infinity_uwb_ros::remote_uwb_info info;
        info.sys_time = this->sys_time;
        info.remote_node_num = this->nodes_info.size();
        info.self_id = this->self_id;
        info.header.stamp = ros::Time::now();
        for (auto k : this->nodes_info)
        {
            int _id = k.first;
            RemoteNodeInfo nod = k.second;
            info.node_ids.push_back(_id);
            info.node_dis.push_back(nod.distance);
            info.recv_distance_time.push_back(nod.dis_time);
            info.active.push_back(nod.active);
            info.rssi.push_back(nod.rssi);
            info.data_available.push_back(nod.msg.size()!=0);
            // std::string ret((char*)nod.msg.data(), nod.msg.size());
            // Buffer bu
            infinity_uwb_ros::data_buffer buf;
            buf.data = nod.msg;
            info.datas.push_back(buf);
        }
        remote_node_pub.publish(info);
        if (count ++ % 50 == 1)
        {
            ROS_INFO("[c%d,ts %d] ID %d nodes total %d active %d\n",count, sys_time,self_id, info.remote_node_num, active_node_num);
            fflush(stdout);
        }
    }
private:
    ros::Publisher remote_node_pub, broadcast_data_pub;
    ros::Subscriber recv_bdmsg;
};

int main(int argc, char** argv)
{
    ROS_INFO("ININITY UWB ROS\nIniting\n");
    
    ros::init(argc, argv, "uwb_node");

    ros::NodeHandle nh("uwb_node");
    int baudrate = 921600;
    std::string serial_name;
    nh.param<int>("baudrate", baudrate, 921600);
    nh.param<std::string>("serial_name", serial_name, "/dev/ttyUSB0");

    UWBRosNodeofNode uwbhelper(serial_name, 921600, nh, true);


    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        uwbhelper.read_and_parse();
        ros::spinOnce();
        loop_rate.sleep();
    }
}