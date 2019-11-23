#include "ros/ros.h"
#include <iostream>
#include "loop_net.h"
#include "loop_cam.h"
#include "loop_detector.h"
#include <chrono> 
#include <Eigen/Eigen>

using namespace std::chrono; 

double DT_MS(system_clock::time_point start) {
    return duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0;
}

class SwarmLoopNode {
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
    
    
    void VIOKF_callback(const vins::VIOKeyframe & viokf) {
        
        Eigen::Vector3d drone_pos(viokf.pose_drone.position.x, viokf.pose_drone.position.y, viokf.pose_drone.position.z);
        double dpos = (last_keyframe_position - drone_pos).norm();
        int keyframe_size = viokf.feature_points_2d_norm.size();

        if ( dpos < min_movement_keyframe || keyframe_size < MIN_KEYFEATURES) {
            ROS_INFO("THROW Keyframe MOVE %3.2fm LANDMARK %d", dpos, keyframe_size);
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

public:
    SwarmLoopNode(ros::NodeHandle& nh) {
        //Init Loop Net
        int _bport = -1;
        std::string _broadcast_ip = "0.0.0.0";
        std::string camera_config_path = "";
        std::string BRIEF_PATTHER_FILE = "";
        std::string ORB_VOC = "";

        nh.param<int>("broadcast_port", _bport, 9988);
        nh.param<double>("min_movement_keyframe", min_movement_keyframe, 0.3);

        nh.param<std::string>("broadcast_ip", _broadcast_ip, 
            "192.168.63.255");
        nh.param<std::string>("camera_config_path",camera_config_path, 
            "/home/xuhao/swarm_ws/src/VINS-Fusion-gpu/config/vi_car/cam0_mei.yaml");
        nh.param<std::string>("BRIEF_PATTHER_FILE", BRIEF_PATTHER_FILE, 
            "/home/xuhao/swarm_ws/src/VINS-Fusion-gpu/support_files/brief_pattern.yml");

        nh.param<std::string>("BRIEF_PATTHER_FILE", ORB_VOC, 
            "/home/xuhao/swarm_ws/src/swarm_localization/support_files/ORBvoc.txt");

        nh.param<bool>("debug_image", debug_image, false);
        
        loop_net = new LoopNet(_broadcast_ip, _bport);
        loop_cam = new LoopCam(camera_config_path, BRIEF_PATTHER_FILE);
        loop_detector = new LoopDetector(ORB_VOC);
        loop_detector->loop_cam = loop_cam;

        camera_sub = nh.subscribe("left_camera", 1000, &SwarmLoopNode::image_callback, this);
        viokeyframe_sub = nh.subscribe("/vins_estimator/viokeyframe", 1000, &SwarmLoopNode::VIOKF_callback, this);
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
    ros::spin();

    return 0;
}
