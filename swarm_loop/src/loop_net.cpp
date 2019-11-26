#include "loop_net.h"

void LoopNet::setup_network(std::string _lcm_uri) {
    if (!lcm.good()) {
        ROS_ERROR("LCM %s failed", _lcm_uri.c_str());
        exit(-1);
    }
    lcm.subscribe("SWARM_LOOP_IMG_DES", &LoopNet::on_img_desc_recevied, this);
    lcm.subscribe("SWARM_LOOP_CONN", &LoopNet::on_loop_connection_recevied, this);
}

void LoopNet::broadcast_img_desc(const ImageDescriptor_t & img_des) {
    ROS_INFO("Broadcast Loop Image");
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
    printf("Image desc recevied\n");
    fflush(stdout);
    this->img_desc_callback(*msg);
}


void LoopNet::on_loop_connection_recevied(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const LoopConnection_t* msg) {
    ROS_INFO("Received Loop Connection from LCM!!!");
    printf("Loop connection recevied\n");
    
}