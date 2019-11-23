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
    
using namespace swarm_msgs;
using namespace camodocal;



class LoopCam {
    int cam_count = 0;
    int loop_duration = 10;

    std::vector<cv::Mat> image_queue;
    std::vector<double> image_ts_queue;

public:
    // LoopDetector * loop_detector = nullptr;

    LoopCam(const std::string & _camera_config_path, const std::string & BRIEF_PATTERN_FILE);
    void on_camera_message(const sensor_msgs::ImageConstPtr& msg);
    std::pair<ImageDescriptor_t, cv::Mat> on_keyframe_message(const vins::VIOKeyframe& msg);

    cv::Mat pop_image_ts(ros::Time ts);
    ImageDescriptor_t feature_detect(const cv::Mat & _img);
    cv::Mat landmark_desc_compute(const cv::Mat & _img, const std::vector<geometry_msgs::Point32> & points_uv);

    std::vector<cv::Point2f> project_to_image(std::vector<cv::Point2f> points_norm2d);

private:
    CameraPtr cam;
};
