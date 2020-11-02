#pragma once
#include "ros/ros.h"
#include <iostream>
#include "loop_net.h"
#include "loop_cam.h"
#include "loop_detector.h"
#include <chrono> 
#include <Eigen/Eigen>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <mutex>

using namespace std::chrono; 

namespace swarm_localization_pkg {
class SwarmLoop {
protected:
    LoopDetector * loop_detector = nullptr;
    LoopCam * loop_cam = nullptr;
    LoopNet * loop_net = nullptr;
    ros::Subscriber cam_sub;
    bool debug_image = false;
    double min_movement_keyframe = 0.3;
    int self_id = 0;
    bool recived_image = false;
    ros::Time last_kftime;
    Eigen::Vector3d last_keyframe_position = Eigen::Vector3d(10000, 10000, 10000);

    std::set<ros::Time> received_keyframe_stamps;


    void on_loop_connection (LoopConnection & loop_con, bool is_local = false);

    std::queue<vins::FlattenImages> viokfs;
    std::mutex viokf_lock;

    vins::FlattenImages find_viokf(const nav_msgs::Odometry & odometry);

    void flatten_raw_callback(const vins::FlattenImages & viokf);

    double last_invoke = 0;
    
    void odometry_callback(const nav_msgs::Odometry & odometry);

    void odometry_keyframe_callback(const nav_msgs::Odometry & odometry);

    void VIOnonKF_callback(const vins::FlattenImages & viokf);

    void VIOKF_callback(const vins::FlattenImages & viokf, bool nonkeyframe = false);

    void on_remote_image_ros(const swarm_msgs::ImageDescriptor & remote_img_desc);

    void on_remote_image(const FisheyeFrameDescriptor_t & frame_desc);

    ros::Subscriber camera_sub;
    ros::Subscriber viokeyframe_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber keyframe_odometry_sub;
    ros::Subscriber flatten_raw_sub;
    ros::Subscriber remote_img_sub;
    ros::Subscriber viononkeyframe_sub;
    ros::Publisher loopconn_pub;
    ros::Publisher remote_image_desc_pub;
    bool enable_pub_remote_img;
    bool enable_sub_remote_img;
    bool send_img;
    bool send_whole_img_desc;
    std::thread th;

    double max_freq = 1.0;
    double recv_msg_duration = 0.5;
    double superpoint_thres = 0.012;

    ros::Timer timer;
public:
    SwarmLoop ();
    
protected:
    virtual void Init(ros::NodeHandle & nh);
};

}