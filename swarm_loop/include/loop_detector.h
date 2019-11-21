#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <swarm_msgs/ImageDescriptor.h>
#include <swarm_msgs/LoopConnection.h>
#include <swarm_msgs/ImageDescriptor_t.hpp>

using namespace swarm_msgs;

class LoopDetector {
public:
    LoopDetector();
    void on_image_recv(const ImageDescriptor_t & img_des);
    void on_loop_connection(const LoopConnection & loop_conn);
};
