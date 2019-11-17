#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <swarm_msgs/ImageDescriptor.h>
#include <swarm_msgs/LoopConnection.h>
#include <camodocal/camera_models/Camera.h>
#include <functional>
#include <loop_detector.h>
#include <vins/VIOKeyframe.h>
    
using namespace swarm_msgs;
using namespace camodocal;

#define FAST_THRES (20.0f)
#define ORB_FEATURE_SIZE (32) // For ORB
#define LOOP_FEATURE_NUM (1000)

class LoopCam {
    int cam_count = 0;
    int loop_duration = 10;

    std::vector<cv::Mat> image_queue;

public:
    LoopDetector * loop_detector = nullptr;

    LoopCam(const std::string & _camera_config_path, const std::string & BRIEF_PATTERN_FILE);
    void on_camera_message(const sensor_msgs::ImageConstPtr& msg);
    void on_keyframe_message(const vins::VIOKeyframe& msg);

    cv::Mat & pop_image_ts(ros::Time ts);
    ImageDescriptor feature_detect(const cv::Mat & _img);

private:
    CameraPtr cam;
};
