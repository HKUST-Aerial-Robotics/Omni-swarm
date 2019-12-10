#include "ros/ros.h"
#include <iostream>
#include "loop_net.h"
#include "loop_cam.h"
#include "loop_detector.h"
#include <chrono> 
#include <Eigen/Eigen>
#include <thread>

using namespace std::chrono; 

class SwarmLoopSpy {
public:
    LoopNet * loop_net = nullptr;
    LoopCam * loop_cam = nullptr;
    std::map<int, ImageDescriptor_t> all_images;
    ros::Timer timer;
public:
    SwarmLoopSpy(ros::NodeHandle& nh) {
        //Init Loop Net
        std::string _lcm_uri = "0.0.0.0";
        std::string camera_config_path = "";
        std::string BRIEF_PATTHER_FILE = "";
        std::string ORB_VOC = "";
        int self_id = -1;
        nh.param<std::string>("camera_config_path",camera_config_path, 
            "/home/xuhao/swarm_ws/src/VINS-Fusion-gpu/config/vi_car/cam0_mei.yaml");

        loop_cam = new LoopCam(camera_config_path, BRIEF_PATTHER_FILE, self_id, nh);

        nh.param<std::string>("lcm_uri", _lcm_uri, "udpm://224.0.0.251:7667?ttl=1");
        loop_net = new LoopNet(_lcm_uri);
        loop_net->img_desc_callback = [&] (const ImageDescriptor_t & img_desc) {
            ROS_INFO("Received Img Desc from %d", img_desc.drone_id);
            all_images[img_desc.drone_id] = img_desc;
        };
        timer = nh.createTimer(ros::Duration(0.03), &SwarmLoopSpy::timer_callback, this);
    }

      void on_loop_connection (LoopConnection & loop_con, bool is_local = false) {
        ROS_INFO("Loop conn from %d to %d", loop_con.id_a, loop_con.id_b);
    }

    void timer_callback(const ros::TimerEvent & e) {
        for (auto &it : all_images) {
            auto & img_desc = it.second;
            char win_name[100] = {0};
            char frame_name[100] = {0};
            sprintf(win_name, "Drone%d", img_desc.drone_id);
            auto ret = cv::imdecode(img_desc.image, cv::IMREAD_GRAYSCALE);
            cv::cvtColor(ret, ret, cv::COLOR_GRAY2BGR);
            sprintf(frame_name, "Frame %d", img_desc.msg_id);
            cv::putText(ret, frame_name, cv::Point2f(10,10), cv::FONT_HERSHEY_PLAIN, 0.8,  cv::Scalar(255,0,255,255));

            auto nowPts = toCV(img_desc.landmarks_2d);

            for (auto pt: nowPts) {
                // std::cout << pt << std::endl;
                cv::circle(ret, pt/2, 1, cv::Scalar(0,0, 255),1);
            }

            cv::resize(ret, ret, cv::Size(), 4, 4);
            cv::imshow(win_name, ret);
        }
        cv::waitKey(10);
    }

};

int main(int argc, char **argv) {
    ROS_INFO("SWARM_LOOP INIT");
    srand(time(NULL));

    ros::init(argc, argv, "swarm_loop_spy");
    ros::NodeHandle nh("swarm_loop_spy");
    SwarmLoopSpy loopnode(nh);

    std::thread thread([&] {
        while(0 == loopnode.loop_net->lcm_handle()) {
        }
    });
    ros::spin();

    return 0;
}
