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
    

#ifdef USE_DEEPNET
#include <tx2_whole_image_desc_server/WholeImageDescriptorComputeTS.h>
using namespace tx2_whole_image_desc_server;
#endif

using namespace swarm_msgs;
using namespace camodocal;



class LoopCam {
    int cam_count = 0;
    int loop_duration = 10;
    int self_id = 0;
    std::vector<cv::Mat> image_queue;
    std::vector<double> image_ts_queue;

#ifdef USE_DEEPNET
    ros::ServiceClient deepnet_client;
#endif

public:
    // LoopDetector * loop_detector = nullptr;

    LoopCam(const std::string & _camera_config_path, const std::string & BRIEF_PATTERN_FILE, int self_id, ros::NodeHandle & nh);
    void on_camera_message(const sensor_msgs::ImageConstPtr& msg);
    std::pair<ImageDescriptor_t, cv::Mat> on_keyframe_message(const vins::VIOKeyframe& msg);

    cv::Mat pop_image_ts(ros::Time ts);
    ImageDescriptor_t extractor_img_desc(const cv::Mat & _img);
    ImageDescriptor_t extractor_img_desc_deepnet(ros::Time stamp);
    cv::Mat landmark_desc_compute(const cv::Mat & _img, const std::vector<geometry_msgs::Point32> & points_uv);

    std::vector<cv::Point2f> project_to_image(std::vector<cv::Point2f> points_norm2d);

    cv::Point2d project_to_norm2d(cv::Point2f p);

    void encode_image(cv::Mat & _img, ImageDescriptor_t & _img_desc);

private:
    CameraPtr cam;
};
