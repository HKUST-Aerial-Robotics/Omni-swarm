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

public:
    SwarmLoopSpy(ros::NodeHandle& nh) {
        //Init Loop Net
        std::string _lcm_uri = "0.0.0.0";
        std::string camera_config_path = "";
        std::string BRIEF_PATTHER_FILE = "";
        std::string ORB_VOC = "";

        nh.param<std::string>("lcm_uri", _lcm_uri, "udpm://224.0.0.251:7667?ttl=1");
        loop_net = new LoopNet(_lcm_uri);
        loop_net->img_desc_callback = [&] (const ImageDescriptor_t & img_desc) {
            char win_name[100] = {0};
            sprintf(win_name, "Drone%d", img_desc.drone_id);
            auto ret = cv::imdecode(img_desc.image, cv::IMREAD_GRAYSCALE);
            cv::imshow(win_name, ret);
            cv::waitKey(30);
        };
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
