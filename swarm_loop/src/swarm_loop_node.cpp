#include "ros/ros.h"
#include <iostream>
#include "loop_net.h"
#include "loop_cam.h"
#include "loop_detector.h"
#include <chrono> 
#include <Eigen/Eigen>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <mutex>

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
    backward::SignalHandling sh;
}

using namespace std::chrono; 

double DT_MS(system_clock::time_point start) {
    return duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0;
}

namespace swarm_localization_pkg {
class SwarmLoopNode  : public nodelet::Nodelet {
public:
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


    void on_loop_connection (LoopConnection & loop_con, bool is_local = false) {
        if(is_local) {
            loop_net->broadcast_loop_connection(loop_con);
        }

        // ROS_INFO("Pub loop conn. is local %d", is_local);
        loopconn_pub.publish(loop_con);
    }

    void VIOnonKF_callback(const vins::VIOKeyframe & viokf) {
        //If never received image or 15 sec not receiving kf, use this as KF, this is ensure we don't missing data
        //Note that for the second case, we will not add it to database, matching only
            
        if (!recived_image && (viokf.header.stamp - last_kftime).toSec() > INIT_ACCEPT_NONKEYFRAME_WAITSEC) {
            //
            ROS_INFO("USE non vio kf as KF at first!");
            return;
        }

        if ((viokf.header.stamp - last_kftime).toSec() > ACCEPT_NONKEYFRAME_WAITSEC) {
            //
        }
    }

    vins::FlattenImages viokf;
    std::mutex viokf_lock;
    void flatten_raw_callback(const vins::FlattenImages & viokf) {
        viokf_lock.lock();
        this->viokf = viokf;
        viokf_lock.unlock();
    }

    double last_invoke = 0;
    void odometry_callback(const nav_msgs::Odometry & odometry) {
        viokf_lock.lock();
        if (abs(odometry.header.stamp.toSec() - viokf.header.stamp.toSec()) > 1e-3 ||
            odometry.header.stamp.toSec() - last_invoke < 1/max_freq) {
            viokf_lock.unlock();
            return;
        }

        ROS_INFO("New KF %fms", odometry.header.stamp.toSec() - last_invoke );

        last_invoke = odometry.header.stamp.toSec();
        viokf.pose_drone = odometry.pose.pose;
        vins::FlattenImages _viokf = viokf;

        viokf_lock.unlock();

        if ((viokf.header.stamp - last_kftime).toSec() > ACCEPT_NONKEYFRAME_WAITSEC) {
            VIOKF_callback(_viokf, true);
        } else {
            VIOKF_callback(_viokf);
        }
    }

    void VIOKF_callback(const vins::FlattenImages & viokf, bool accept_non_movement = false) {
        Eigen::Vector3d drone_pos(viokf.pose_drone.position.x, viokf.pose_drone.position.y, viokf.pose_drone.position.z);
        double dpos = (last_keyframe_position - drone_pos).norm();
        bool is_non_kf = false;
        if (dpos < min_movement_keyframe) {
            if(!accept_non_movement) {
                ROS_WARN("VIOKF no enough movement, will giveup");
                return;
            } else {
                // is_non_kf = true;
                ROS_INFO("ADD VIONonKeyframe MOVE %3.2fm", dpos);
            }

        } else {
            ROS_INFO("ADD VIOKeyframe MOVE %3.2fm", dpos);
        }

        last_kftime = viokf.header.stamp;

        auto start = high_resolution_clock::now();
        cv::Mat img;
        
        auto ret = loop_cam->on_flattened_images(viokf, img);
        
        ret.prevent_adding_db = is_non_kf;

        if (ret.landmark_num == 0) {
            ROS_WARN("Null img desc, CNN no ready");
            return;
        }

        recived_image = true;
        last_keyframe_position = drone_pos;

        // std::cout << "Cam Cost " << DT_MS(start) << "ms" << std::endl;
        
        loop_net->broadcast_img_desc(ret);

        loop_detector->on_image_recv(ret, img);

        // std::cout << "Cam+LD Cost " << DT_MS(start) << "ms" <<  std::endl;
    }

    void on_remote_image_ros(const swarm_msgs::ImageDescriptor & remote_img_desc) {
        ROS_INFO("Remote");
        this->on_remote_image(toLCMImageDescriptor(remote_img_desc));
    }

    void on_remote_image(const ImageDescriptor_t & img_desc) {
        loop_detector->on_image_recv(img_desc);
    }

    ros::Subscriber camera_sub;
    ros::Subscriber viokeyframe_sub;
    ros::Subscriber odometry_sub;
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

    ros::Timer timer;
public:
    SwarmLoopNode () {}
    
private:
    virtual void onInit() {
        //Init Loop Net
        auto nh = getMTPrivateNodeHandle();
        std::string _lcm_uri = "0.0.0.0";
        std::string camera_config_path = "";
        std::string BRIEF_PATTHER_FILE = "";
        std::string ORB_VOC = "";

        nh.param<int>("self_id", self_id, -1);
        nh.param<double>("min_movement_keyframe", min_movement_keyframe, 0.3);

        nh.param<std::string>("lcm_uri", _lcm_uri, "udpm://224.0.0.251:7667?ttl=1");
        
        nh.param<int>("loop_image_downsample", LOOP_IMAGE_DOWNSAMPLE, 1);
        nh.param<int>("init_loop_min_feature_num", INIT_MODE_MIN_LOOP_NUM, 10);
        nh.param<int>("init_loop_min_feature_num_l2", INIT_MODE_MIN_LOOP_NUM_LEVEL2, 10);
        nh.param<int>("match_index_dist", MATCH_INDEX_DIST, 10);
        nh.param<int>("min_loop_feature_num", MIN_LOOP_NUM, 15);
        nh.param<int>("jpg_quality", JPG_QUALITY, 50);
        nh.param<int>("accept_min_3d_pts", ACCEPT_MIN_3D_PTS, 50);
        nh.param<bool>("enable_lk", ENABLE_LK_LOOP_DETECTION, true);
        nh.param<bool>("enable_pub_remote_img", enable_pub_remote_img, false);
        nh.param<bool>("enable_sub_remote_img", enable_sub_remote_img, false);
        nh.param<bool>("send_img", send_img, false);
        nh.param<bool>("send_whole_img_desc", send_whole_img_desc, false);
        nh.param<double>("query_thres", INNER_PRODUCT_THRES, 0.6);
        nh.param<double>("init_query_thres", INIT_MODE_PRODUCT_THRES, 0.3);
        nh.param<double>("min_movement_keyframe", MIN_MOVEMENT_KEYFRAME, 0.2);
        nh.param<double>("max_freq", max_freq, 1.0);
        nh.param<double>("recv_msg_duration", recv_msg_duration, 0.5);

        nh.param<std::string>("camera_config_path",camera_config_path, 
            "/home/xuhao/swarm_ws/src/VINS-Fusion-gpu/config/vi_car/cam0_mei.yaml");
        nh.param<std::string>("BRIEF_PATTHER_FILE", BRIEF_PATTHER_FILE, 
            "/home/xuhao/swarm_ws/src/VINS-Fusion-gpu/support_files/brief_pattern.yml");
        nh.param<std::string>("ORB_VOC", ORB_VOC, 
            "/home/xuhao/swarm_ws/src/swarm_localization/support_files/ORBvoc.txt");

        nh.param<bool>("debug_image", debug_image, false);
        
        loop_net = new LoopNet(_lcm_uri, send_img, send_whole_img_desc, recv_msg_duration);
        loop_cam = new LoopCam(camera_config_path, BRIEF_PATTHER_FILE, self_id, send_img, nh);
        loop_cam->show = debug_image; 
#ifdef USE_DEEPNET
        loop_detector = new LoopDetector();
#else
        loop_detector = new LoopDetector(ORB_VOC);
#endif
        loop_detector->self_id = self_id;
        loop_detector->loop_cam = loop_cam;
        loop_detector->enable_visualize = debug_image;

        loop_detector->on_loop_cb = [&] (LoopConnection & loop_con) {
            this->on_loop_connection(loop_con, true);
        };

        loop_net->img_desc_callback = [&] (const ImageDescriptor_t & img_desc) {
            if (enable_pub_remote_img) {
                remote_image_desc_pub.publish(toROSImageDescriptor(img_desc));
            }

            this->on_remote_image(img_desc);
        };

        loop_net->loopconn_callback = [&] (const LoopConnection_t & loop_conn) {
            auto loc = toROSLoopConnection(loop_conn);
            on_loop_connection(loc, false);
        };

        flatten_raw_sub = nh.subscribe("/vins_estimator/flattened_gray", 1, &SwarmLoopNode::flatten_raw_callback, this, ros::TransportHints().tcpNoDelay());
        odometry_sub  = nh.subscribe("/vins_estimator/odometry", 1, &SwarmLoopNode::odometry_callback, this, ros::TransportHints().tcpNoDelay());
        loopconn_pub = nh.advertise<swarm_msgs::LoopConnection>("loop_connection", 10);
        
        if (enable_sub_remote_img) {
            ROS_INFO("Subscribing remote image from bag");
            remote_img_sub = nh.subscribe("/swarm_loop/remote_image_desc", 1, &SwarmLoopNode::on_remote_image_ros, this, ros::TransportHints().tcpNoDelay());
        }

        if (enable_pub_remote_img) {
            remote_image_desc_pub = nh.advertise<swarm_msgs::ImageDescriptor>("remote_image_desc", 10);
        }

        timer = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent & e) {
            loop_net->scan_recv_packets();
        });

        th = std::thread([&] {
            while(0 == loop_net->lcm_handle()) {
            }
       });
    }
};

PLUGINLIB_EXPORT_CLASS(swarm_localization_pkg::SwarmLoopNode, nodelet::Nodelet);
}