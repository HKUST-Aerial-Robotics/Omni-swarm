#include "loop_net.h"

void LoopNet::setup_network(std::string _lcm_uri) {
    if (!lcm.good()) {
        ROS_ERROR("LCM %s failed", _lcm_uri.c_str());
        exit(-1);
    }
}

void LoopNet::broadcast_img_desc(const ImageDescriptor_t & img_des) {
    lcm.publish("SWARM_LOOP_IMG_DES", &img_des);

}

void LoopNet::broadcast_loop_connection(const LoopConnection & loop_conn) {
    auto _loop_conn = toLCMLoopConnection(loop_conn);
    lcm.publish("SWARM_LOOP_CONN", &_loop_conn);
}

void LoopNet::on_img_desc_recevied(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const ImageDescriptor_t* msg) {

    ROS_INFO("Received image desc from LCM!!!");

}


void LoopNet::on_loop_connection_recevied(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const LoopConnection_t* msg) {
    ROS_INFO("Received Loop Connection from LCM!!!");
}