#include "ros/ros.h"
#include <iostream>
#include "loop_net.h"
#include "loop_cam.h"
#include "loop_detector.h"

class SwarmLoopNode {
LoopDetector * loop_detector = nullptr;
LoopCam * loop_cam = nullptr;
LoopNet * loop_net = nullptr;

ros::Subscriber cam_sub;
public:
    SwarmLoopNode(ros::NodeHandle& nh)
    {
        //Init Loop Net
        int _bport = -1;
        std::string _broadcast_ip = "0.0.0.0";
        nh.param<int>("broadcast_port", _bport, 9988);
        nh.param<std::string>("broadcast_ip", _broadcast_ip, "192.168.63.255");
        loop_net = new LoopNet(_broadcast_ip, _bport);
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
