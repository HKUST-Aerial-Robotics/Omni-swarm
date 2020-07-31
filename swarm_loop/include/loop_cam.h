#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <swarm_msgs/ImageDescriptor.h>
#include <swarm_msgs/LoopConnection.h>
#include <camodocal/camera_models/Camera.h>
#include <functional>
#include <vins/VIOKeyframe.h>
#include <swarm_msgs/ImageDescriptor_t.hpp>
#include "loop_defines.h"
#include <swarm_loop/HFNetSrv.h>
#include <vins/FlattenImages.h>

using namespace swarm_msgs;
using namespace camodocal;
using namespace swarm_loop;


class LoopCam {
    int cam_count = 0;
    int loop_duration = 10;
    int self_id = 0;

    ros::ServiceClient deepnet_client;

public:
    // LoopDetector * loop_detector = nullptr;

    LoopCam(const std::string & _camera_config_path, const std::string & BRIEF_PATTERN_FILE, int self_id, ros::NodeHandle & nh);

    ImageDescriptor_t on_flattened_images(const vins::FlattenImages& msg, cv::Mat & img, const int & vcam_id = 1);

    ImageDescriptor_t extractor_img_desc_deepnet(ros::Time stamp, const sensor_msgs::Image& msg);

    cv::Mat landmark_desc_compute(const cv::Mat & _img, const std::vector<geometry_msgs::Point32> & points_uv);

    std::vector<cv::Point2f> project_to_image(std::vector<cv::Point2f> points_norm2d);

    cv::Point2d project_to_norm2d(cv::Point2f p);

    void encode_image(cv::Mat & _img, ImageDescriptor_t & _img_desc);

private:
    CameraPtr cam;
};
