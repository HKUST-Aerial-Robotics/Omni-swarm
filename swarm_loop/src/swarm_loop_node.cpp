#include "ros/ros.h"
#include <iostream>
#include "loop_net.h"
#include "loop_cam.h"
#include "loop_detector.h"
#include <chrono> 
#include <Eigen/Eigen>
#include <thread>

using namespace std::chrono; 

double DT_MS(system_clock::time_point start) {
    return duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0;
}

class SwarmLoopNode {
public:
    LoopDetector * loop_detector = nullptr;
    LoopCam * loop_cam = nullptr;
    LoopNet * loop_net = nullptr;
    ros::Subscriber cam_sub;
    bool debug_image = false;
    double min_movement_keyframe = 0.3;

    Eigen::Vector3d last_keyframe_position = Eigen::Vector3d(10000, 10000, 10000);

    void image_callback(const sensor_msgs::ImageConstPtr& msg) {
        loop_cam->on_camera_message(msg);
    }
    

    void on_loop_connection (LoopConnection & loop_con, bool is_local = false) {
        if(is_local) {
            loop_net->broadcast_loop_connection(loop_con);
        }
        loopconn_pub.publish(loop_con);
    } 
    
    void VIOKF_callback(const vins::VIOKeyframe & viokf) {
        
        Eigen::Vector3d drone_pos(viokf.pose_drone.position.x, viokf.pose_drone.position.y, viokf.pose_drone.position.z);
        double dpos = (last_keyframe_position - drone_pos).norm();
        int keyframe_size = viokf.feature_points_2d_norm.size();

        if ( dpos < min_movement_keyframe || keyframe_size < MIN_KEYFEATURES) {
            // ROS_INFO("THROW Keyframe MOVE %3.2fm LANDMARK %d", dpos, keyframe_size);
            return;
        } else {
            last_keyframe_position = drone_pos;
            ROS_INFO("ADD Keyframe MOVE %3.2fm LANDMARK %d ", dpos, keyframe_size);
        }

        auto start = high_resolution_clock::now();
        auto ret = loop_cam->on_keyframe_message(viokf);
        if (ret.first.landmark_num == 0) {
            return;
        }

        std::cout << "Cam Cost " << DT_MS(start) << "ms" << std::endl;
        
        if (ret.first.landmark_num > MIN_LOOP_NUM) {
            loop_net->broadcast_img_desc(ret.first);
        }

        //Check ides vaild
        if (debug_image) {
            loop_detector->on_image_recv(ret.first, ret.second);
        } else {
            loop_detector->on_image_recv(ret.first);
        }

        std::cout << "Cam+LD Cost " << DT_MS(start) << "ms" <<  std::endl;
    }

    ros::Subscriber camera_sub;
    ros::Subscriber viokeyframe_sub;
    ros::Publisher loopconn_pub;

public:
    SwarmLoopNode(ros::NodeHandle& nh) {
        //Init Loop Net
        int _bport = -1;
        std::string _lcm_uri = "0.0.0.0";
        std::string camera_config_path = "";
        std::string BRIEF_PATTHER_FILE = "";
        std::string ORB_VOC = "";

        nh.param<int>("broadcast_port", _bport, 9988);
        nh.param<double>("min_movement_keyframe", min_movement_keyframe, 0.3);

        nh.param<std::string>("lcm_uri", _lcm_uri, "udpm://224.0.0.251:7667?ttl=1");
        nh.param<std::string>("camera_config_path",camera_config_path, 
            "/home/xuhao/swarm_ws/src/VINS-Fusion-gpu/config/vi_car/cam0_mei.yaml");
        nh.param<std::string>("BRIEF_PATTHER_FILE", BRIEF_PATTHER_FILE, 
            "/home/xuhao/swarm_ws/src/VINS-Fusion-gpu/support_files/brief_pattern.yml");

        nh.param<std::string>("BRIEF_PATTHER_FILE", ORB_VOC, 
            "/home/xuhao/swarm_ws/src/swarm_localization/support_files/ORBvoc.txt");

        nh.param<bool>("debug_image", debug_image, false);
        
        loop_net = new LoopNet(_lcm_uri);
        loop_cam = new LoopCam(camera_config_path, BRIEF_PATTHER_FILE);
        loop_detector = new LoopDetector(ORB_VOC);
        loop_detector->loop_cam = loop_cam;
        loop_detector->enable_visualize = debug_image;

        loop_detector->on_loop_cb = [&] (LoopConnection & loop_con) {
            this->on_loop_connection(loop_con, true);
        };

        loop_net->img_desc_callback = [&] (const ImageDescriptor_t & img_desc) {
            loop_detector->on_image_recv(img_desc);
        };

        loop_net->loopconn_callback = [&] (const LoopConnection_t & loop_conn) {
            auto loc = toROSLoopConnection(loop_conn);
            on_loop_connection(loc, false);
        };

        camera_sub = nh.subscribe("left_camera", 1000, &SwarmLoopNode::image_callback, this);
        viokeyframe_sub = nh.subscribe("/vins_estimator/viokeyframe", 1000, &SwarmLoopNode::VIOKF_callback, this);
        loopconn_pub = nh.advertise<swarm_msgs::LoopConnection>("loop_connection", 1);
    }
};

int main(int argc, char **argv) {
    ROS_INFO("SWARM_LOOP INIT");
    srand(time(NULL));

    ros::init(argc, argv, "swarm_loop");
    ros::NodeHandle nh("swarm_loop");
    SwarmLoopNode loopnode(nh);

    //ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    //spinner.spin();
    std::thread thread([&] {
        while(0 == loopnode.loop_net->lcm_handle()) {
        }
    });
    ros::spin();

    return 0;
}
