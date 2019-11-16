#pragma once

#include <ros/ros.h>
#include <swarm_msgs/ImageDescriptor.h>
#include <swarm_msgs/LoopConnection.h>
#include <string>

using namespace swarm_msgs;

class LoopNet {
    std::string broadcast_ip;
    int port = 9988;
public:
    LoopNet(std::string _broadcast_ip, int _port):
        broadcast_ip(_broadcast_ip), port(_port)
    {
        setup_network(broadcast_ip, port);
    }

    void setup_network(std::string _broadcast_ip, int _ip);
    void broadcast_img_des(const ros::Time & stamp, const ImageDescriptor & img_des);
    void broadcast_loop_connection(const LoopConnection & loop_conn);
};
