#include "loop_net.h"

void LoopNet::setup_network(std::string _broadcast_ip, int _ip) {
    if (!lcm.good()) {
        ROS_ERROR("LCM failed");
        exit(-1);
    }
}

void LoopNet::broadcast_img_des(const ros::Time & stamp, const ImageDescriptor_t & img_des) {
    lcm.publish("SWARM_LOOP_IMG_DES", &img_des);

}

void LoopNet::broadcast_loop_connection(const LoopConnection & loop_conn) {
    
}