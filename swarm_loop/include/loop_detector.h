#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <swarm_msgs/ImageDescriptor.h>
#include <swarm_msgs/LoopConnection.h>
#include <swarm_msgs/ImageDescriptor_t.hpp>
#include <DBoW3/DBoW3.h>
#include "loop_defines.h"
#include <loop_cam.h>
#include <functional>

using namespace swarm_msgs;

class LoopDetector {

protected:
    DBoW3::Vocabulary voc;
    DBoW3::Database db;
    std::map<unsigned int, cv::Mat> id2imgs;
    std::map<unsigned int, ImageDescriptor_t> id2imgdes;
    std::vector<cv::Scalar> colors;
    bool compute_loop(const unsigned int & _img_index_now, const unsigned int & _img_index_old, LoopConnection & ret);

public:
    std::function<void(const LoopConnection &)> on_loop_cb;

    LoopDetector(const std::string & voc_path);
    void on_image_recv(const ImageDescriptor_t & img_des, cv::Mat img=cv::Mat());
    void on_loop_connection(const LoopConnection & loop_conn);
    LoopCam * loop_cam = nullptr;
    bool enable_visualize = true;
    cv::Mat decode_image(const ImageDescriptor_t & _img_desc);
};
