#include <swarm_loop.h>

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

int MIN_DIRECTION_LOOP;
double DETECTOR_MATCH_THRES;

using namespace std::chrono; 

inline double DT_MS(system_clock::time_point start) {
    return duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0;
}

namespace swarm_localization_pkg {

void SwarmLoop::on_loop_connection (LoopConnection & loop_con, bool is_local) {
    if(is_local) {
        loop_net->broadcast_loop_connection(loop_con);
    }

    // ROS_INFO("Pub loop conn. is local %d", is_local);
    loopconn_pub.publish(loop_con);
}

vins::FlattenImages SwarmLoop::find_viokf(const nav_msgs::Odometry & odometry) {
    auto stamp = odometry.header.stamp;
    vins::FlattenImages ret;
    ret.header.stamp = ros::Time(0);
    viokf_lock.lock();

    while (viokfs.size() > 0 && fabs(stamp.toSec() - viokfs.front().header.stamp.toSec()) > 1e-3) {
        // ROS_INFO("Removing d stamp %f", stamp.toSec() - viokfs.front().header.stamp.toSec());
        viokfs.pop();
    }

    if (viokfs.size() > 0 && fabs(stamp.toSec() - viokfs.front().header.stamp.toSec() < 1e-3)) {
        ret = viokfs.front();
        ret.pose_drone = odometry.pose.pose;
        viokfs.pop();
        // ROS_INFO("VIO KF found, returning...");
        viokf_lock.unlock();
        return ret;
    } 

    viokf_lock.unlock();
    return ret;
}

void SwarmLoop::flatten_raw_callback(const vins::FlattenImages & viokf) {
    viokf_lock.lock();
    // ROS_INFO("Received flatten_raw %f", viokf.header.stamp.toSec());
    viokfs.push(viokf);
    viokf_lock.unlock();
}

void SwarmLoop::odometry_callback(const nav_msgs::Odometry & odometry) {
    if (odometry.header.stamp.toSec() - last_invoke < ACCEPT_NONKEYFRAME_WAITSEC) {
        return;
    }

    auto _viokf = find_viokf(odometry);
    if (_viokf.header.stamp.toSec() > 1000) {
        // ROS_INFO("VIO Non Keyframe callback!!");
        VIOnonKF_callback(_viokf);
    } else {
        ROS_WARN("Flattened images correspond to this Odometry not found: %f", odometry.header.stamp.toSec());
    }
}

void SwarmLoop::odometry_keyframe_callback(const nav_msgs::Odometry & odometry) {
    // ROS_INFO("VIO Keyframe received");
    auto _viokf = find_viokf(odometry);
    if (_viokf.header.stamp.toSec() > 1000) {
        VIOKF_callback(_viokf);
    } else {
        ROS_WARN("Flattened images correspond to this Keyframe not found: %f", odometry.header.stamp.toSec());
    }
}

void SwarmLoop::VIOnonKF_callback(const vins::FlattenImages & viokf) {
    //If never received image or 15 sec not receiving kf, use this as KF, this is ensure we don't missing data
    //Note that for the second case, we will not add it to database, matching only
        
    if (!recived_image && (viokf.header.stamp - last_kftime).toSec() > INIT_ACCEPT_NONKEYFRAME_WAITSEC) {
        //
        ROS_INFO("USE non vio kf as KF at first keyframe!");
        VIOKF_callback(viokf);
        return;
    }

    if ((viokf.header.stamp - last_kftime).toSec() > ACCEPT_NONKEYFRAME_WAITSEC) {
        VIOKF_callback(viokf, true);
    }
}

void SwarmLoop::VIOKF_callback(const vins::FlattenImages & viokf, bool nonkeyframe) {
    if (viokf.header.stamp.toSec() - last_invoke < 1/max_freq) {
        return;
    }

    last_invoke = viokf.header.stamp.toSec();
    Eigen::Vector3d drone_pos(viokf.pose_drone.position.x, viokf.pose_drone.position.y, viokf.pose_drone.position.z);
    double dpos = (last_keyframe_position - drone_pos).norm();

    last_kftime = viokf.header.stamp;

    auto start = high_resolution_clock::now();
    std::vector<cv::Mat> imgs;
    
    auto ret = loop_cam->on_flattened_images(viokf, imgs);
    
    ret.prevent_adding_db = nonkeyframe;

    if (ret.landmark_num == 0) {
        ROS_WARN("Null img desc, CNN no ready");
        return;
    }

    recived_image = true;
    last_keyframe_position = drone_pos;

    loop_net->broadcast_fisheye_desc(ret);
    loop_detector->on_image_recv(ret, imgs);
}

void SwarmLoop::on_remote_frame_ros(const swarm_msgs::FisheyeFrameDescriptor & remote_img_desc) {
    // ROS_INFO("Remote");
    if (recived_image) {
        this->on_remote_image(toLCMFisheyeDescriptor(remote_img_desc));
    }
}

void SwarmLoop::on_remote_image(const FisheyeFrameDescriptor_t & frame_desc) {
    loop_detector->on_image_recv(frame_desc);
}

SwarmLoop::SwarmLoop () {}

void SwarmLoop::Init(ros::NodeHandle & nh) {
    //Init Loop Net
    std::string _lcm_uri = "0.0.0.0";
    std::string camera_config_path = "";
    std::string superpoint_model_path = "";
    std::string netvlad_model_path = "";
    int width;
    int height;
    cv::setNumThreads(1);
    nh.param<int>("self_id", self_id, -1);
    nh.param<double>("min_movement_keyframe", min_movement_keyframe, 0.3);

    nh.param<std::string>("lcm_uri", _lcm_uri, "udpm://224.0.0.251:7667?ttl=1");
    
    nh.param<int>("loop_image_downsample", LOOP_IMAGE_DOWNSAMPLE, 1);
    nh.param<int>("init_loop_min_feature_num", INIT_MODE_MIN_LOOP_NUM, 10);
    nh.param<int>("init_loop_min_feature_num_l2", INIT_MODE_MIN_LOOP_NUM_LEVEL2, 10);
    nh.param<int>("match_index_dist", MATCH_INDEX_DIST, 10);
    nh.param<int>("min_loop_feature_num", MIN_LOOP_NUM, 15);
    nh.param<int>("min_match_per_dir", MIN_MATCH_PRE_DIR, 15);
    nh.param<int>("jpg_quality", JPG_QUALITY, 50);
    nh.param<int>("accept_min_3d_pts", ACCEPT_MIN_3D_PTS, 50);
    nh.param<bool>("enable_lk", ENABLE_LK_LOOP_DETECTION, true);
    nh.param<bool>("enable_pub_remote_frame", enable_pub_remote_frame, false);
    nh.param<bool>("enable_pub_local_frame", enable_pub_local_frame, false);
    nh.param<bool>("enable_sub_remote_frame", enable_sub_remote_frame, false);
    nh.param<bool>("send_img", send_img, false);
    nh.param<bool>("is_pc_replay", IS_PC_REPLAY, false);
    nh.param<bool>("send_whole_img_desc", send_whole_img_desc, false);
    nh.param<bool>("send_all_features", SEND_ALL_FEATURES, false);
    nh.param<double>("query_thres", INNER_PRODUCT_THRES, 0.6);
    nh.param<double>("init_query_thres", INIT_MODE_PRODUCT_THRES, 0.3);
    nh.param<double>("min_movement_keyframe", MIN_MOVEMENT_KEYFRAME, 0.2);
    nh.param<double>("max_freq", max_freq, 1.0);
    nh.param<double>("recv_msg_duration", recv_msg_duration, 0.5);
    nh.param<double>("superpoint_thres", superpoint_thres, 0.012);
    nh.param<double>("detector_match_thres", DETECTOR_MATCH_THRES, 0.9);
    nh.param<bool>("lower_cam_as_main", LOWER_CAM_AS_MAIN, false);

    nh.param<double>("triangle_thres", TRIANGLE_THRES, 0.006);
    nh.param<int>("min_direction_loop", MIN_DIRECTION_LOOP, 3);
    nh.param<int>("width", width, 400);
    nh.param<int>("height", height, 208);

    nh.param<std::string>("camera_config_path",camera_config_path, 
        "/home/xuhao/swarm_ws/src/VINS-Fusion-gpu/config/vi_car/cam0_mei.yaml");
    nh.param<std::string>("superpoint_model_path", superpoint_model_path, "");
    nh.param<std::string>("netvlad_model_path", netvlad_model_path, "");

    nh.param<bool>("debug_image", debug_image, false);
    nh.param<std::string>("output_path", OUTPUT_PATH, "");
    
    loop_net = new LoopNet(_lcm_uri, send_img, send_whole_img_desc, recv_msg_duration);
    loop_cam = new LoopCam(camera_config_path, superpoint_model_path, superpoint_thres, netvlad_model_path, width, height, self_id, send_img, nh);
    loop_cam->show = debug_image; 
    loop_detector = new LoopDetector();
    loop_detector->self_id = self_id;
    loop_detector->loop_cam = loop_cam;
    loop_detector->enable_visualize = debug_image;

    loop_detector->on_loop_cb = [&] (LoopConnection & loop_con) {
        this->on_loop_connection(loop_con, true);
    };

    loop_net->frame_desc_callback = [&] (const FisheyeFrameDescriptor_t & frame_desc) {
        if (recived_image) {
            if (enable_pub_remote_frame) {
                remote_image_desc_pub.publish(toROSFisheyeDescriptor(frame_desc));
            }
            this->on_remote_image(frame_desc);
        }
    };

    loop_net->loopconn_callback = [&] (const LoopConnection_t & loop_conn) {
        auto loc = toROSLoopConnection(loop_conn);
        on_loop_connection(loc, false);
    };

    flatten_raw_sub = nh.subscribe("/vins_estimator/flattened_gray", 1, &SwarmLoop::flatten_raw_callback, this, ros::TransportHints().tcpNoDelay());
    odometry_sub  = nh.subscribe("/vins_estimator/odometry", 1, &SwarmLoop::odometry_callback, this, ros::TransportHints().tcpNoDelay());
    keyframe_odometry_sub  = nh.subscribe("/vins_estimator/keyframe_pose", 1, &SwarmLoop::odometry_keyframe_callback, this, ros::TransportHints().tcpNoDelay());

    loopconn_pub = nh.advertise<swarm_msgs::LoopConnection>("loop_connection", 10);
    
    if (enable_sub_remote_frame) {
        ROS_INFO("Subscribing remote image from bag");
        remote_img_sub = nh.subscribe("/swarm_loop/remote_frame_desc", 1, &SwarmLoop::on_remote_frame_ros, this, ros::TransportHints().tcpNoDelay());
    }

    if (enable_pub_remote_frame) {
        remote_image_desc_pub = nh.advertise<swarm_msgs::FisheyeFrameDescriptor>("remote_frame_desc", 10);
    }

    if (enable_pub_local_frame) {
        local_image_desc_pub = nh.advertise<swarm_msgs::FisheyeFrameDescriptor>("local_frame_desc", 10);
    }
    

    timer = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent & e) {
        loop_net->scan_recv_packets();
    });

    th = std::thread([&] {
        while(0 == loop_net->lcm_handle()) {
        }
    });
}

}
