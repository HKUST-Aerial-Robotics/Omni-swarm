#pragma once

#include <ros/ros.h>
#include <swarm_msgs/ImageDescriptor.h>
#include <swarm_msgs/LoopConnection.h>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <swarm_msgs/ImageDescriptor_t.hpp>
#include "loop_defines.h"
#include <swarm_msgs/swarm_lcm_converter.hpp>

using namespace swarm_msgs;

class LoopNet {
    std::string broadcast_ip;
    int port = 9988;
    lcm::LCM lcm;

public:
    LoopNet(std::string _lcm_uri):
        lcm(_lcm_uri) {
        this->setup_network(_lcm_uri);
    }

    void setup_network(std::string _lcm_uri);
    void broadcast_img_desc(const ImageDescriptor_t & img_des);
    void broadcast_loop_connection(const LoopConnection & loop_conn);

    void on_loop_connection_recevied(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const LoopConnection_t* msg);

    void on_img_desc_recevied(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const ImageDescriptor_t* msg);

    void lcm_handle() {
        lcm.handle();
    }
};
