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
    bool compute_loop(const ImageDescriptor_t & new_img_desc, const unsigned int & _img_index_old, LoopConnection & ret);

    int add_to_database(const ImageDescriptor_t & new_img_desc);
    int query_from_database(const ImageDescriptor_t & new_img_desc, int max_index);

public:
    std::function<void(LoopConnection &)> on_loop_cb;

    LoopDetector(const std::string & voc_path);
    LoopDetector();
    void on_image_recv(const ImageDescriptor_t & img_des, cv::Mat img=cv::Mat());
    void on_loop_connection(LoopConnection & loop_conn);
    LoopCam * loop_cam = nullptr;
    bool enable_visualize = true;
    cv::Mat decode_image(const ImageDescriptor_t & _img_desc);

    int database_size() const;

};
