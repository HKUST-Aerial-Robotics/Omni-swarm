#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <swarm_msgs/ImageDescriptor.h>
#include <swarm_msgs/LoopConnection.h>
#include <swarm_msgs/ImageDescriptor_t.hpp>
#include <DBoW3/DBoW3.h>

using namespace swarm_msgs;

#define LOOP_BOW_THRES 0.015
#define MATCH_INDEX_DIST 50

class LoopDetector {
    DBoW3::Vocabulary voc;
    DBoW3::Database db;
    std::map<unsigned int, cv::Mat> id2imgs;
public:
    LoopDetector(const std::string & voc_path);
    void on_image_recv(const ImageDescriptor_t & img_des, cv::Mat img=cv::Mat());
    void on_loop_connection(const LoopConnection & loop_conn);

    LoopConnection compute_loop(const ImageDescriptor_t & img_now, const ImageDescriptor_t & img_des);
};
