#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <swarm_msgs/ImageDescriptor.h>
#include <swarm_msgs/LoopConnection.h>
#include <camodocal/camera_models/Camera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <functional>
#include <vins/VIOKeyframe.h>
#include <swarm_msgs/ImageDescriptor_t.hpp>
#include <swarm_msgs/FisheyeFrameDescriptor_t.hpp>

#include "loop_defines.h"
#include <vins/FlattenImages.h>
#include "superpoint_tensorrt.h"
#include "mobilenetvlad_tensorrt.h"
//#include <swarm_loop/HFNetSrv.h>

using namespace swarm_msgs;
using namespace camodocal;
// using namespace swarm_loop;


class LoopCam {
    int cam_count = 0;
    int loop_duration = 10;
    int self_id = 0;
    ros::ServiceClient hfnet_client;
    ros::ServiceClient superpoint_client;

#ifdef USE_TENSORRT
    SuperPointTensorRT superpoint_net;
    MobileNetVLADTensorRT netvlad_net;
#endif

    bool send_img;
public:

    bool show = false;

    // LoopDetector * loop_detector = nullptr;

    LoopCam(const std::string & _camera_config_path, 
        const std::string & superpoint_model, double thres, const std::string & netvlad_model, int width, int height, 
        int self_id, bool _send_img, ros::NodeHandle & nh);
    
    ImageDescriptor_t extractor_img_desc_deepnet(ros::Time stamp, const sensor_msgs::Image& msg, bool superpoint_mode=false);
    
    ImageDescriptor_t generate_image_descriptor(const vins::FlattenImages & msg, cv::Mat & img, const int & vcam_id, cv::Mat &_show);
    
    FisheyeFrameDescriptor_t on_flattened_images(const vins::FlattenImages & msg, std::vector<cv::Mat> imgs);

    cv::Mat landmark_desc_compute(const cv::Mat & _img, const std::vector<geometry_msgs::Point32> & points_uv);

    std::vector<cv::Point2f> project_to_image(std::vector<cv::Point2f> points_norm2d);

    cv::Point2d project_to_norm2d(cv::Point2f p);

    void encode_image(const cv::Mat & _img, ImageDescriptor_t & _img_desc);
    
    std::vector<int> match_HFNet_local_features(std::vector<cv::Point2f> & pts_up, std::vector<cv::Point2f> & pts_down, std::vector<float> _desc_up, std::vector<float> _desc_down,
        const cv::Mat & up, const cv::Mat & down);

    CameraPtr cam;
    cv::Mat cameraMatrix;
};
