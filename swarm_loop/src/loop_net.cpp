#include "loop_net.h"
#include <time.h> 

void LoopNet::setup_network(std::string _lcm_uri) {
    if (!lcm.good()) {
        ROS_ERROR("LCM %s failed", _lcm_uri.c_str());
        exit(-1);
    }
    lcm.subscribe("SWARM_LOOP_IMG_DES", &LoopNet::on_img_desc_recevied, this);
    lcm.subscribe("SWARM_LOOP_CONN", &LoopNet::on_loop_connection_recevied, this);

    srand((unsigned)time(NULL)); 
}

void LoopNet::broadcast_img_desc(ImageDescriptor_t & img_des) {
    ROS_INFO("Broadcast Loop Image: size %d", img_des.getEncodedSize());
    if (img_des.getEncodedSize() < 0) {
        ROS_ERROR("WRONG SIZE!!!");
        exit(-1);
    }
    img_des.msg_id = rand() + img_des.timestamp.nsec*RAND_MAX;
    sent_message.insert(img_des.msg_id);
    lcm.publish("SWARM_LOOP_IMG_DES", &img_des);
}

void LoopNet::broadcast_loop_connection(LoopConnection & loop_conn) {
    auto _loop_conn = toLCMLoopConnection(loop_conn);
    _loop_conn.msg_id = rand() + loop_conn.ts_a.nsec*RAND_MAX;

    sent_message.insert(_loop_conn.msg_id);
    lcm.publish("SWARM_LOOP_CONN", &_loop_conn);
}

void LoopNet::on_img_desc_recevied(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const ImageDescriptor_t* msg) {
    
    if (sent_message.find(msg->msg_id) != sent_message.end()) {
        // ROS_INFO("Receive self sent IMG message");
        return;
    }
    
    ROS_INFO("Received image desc from LCM!!!");
    this->img_desc_callback(*msg);
}


void LoopNet::on_loop_connection_recevied(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const LoopConnection_t* msg) {

    if (sent_message.find(msg->msg_id) != sent_message.end()) {
        // ROS_INFO("Receive self sent Loop message");
        return;
    }
    ROS_INFO("Received Loop Connection from LCM!!!");    
    loopconn_callback(*msg);
}