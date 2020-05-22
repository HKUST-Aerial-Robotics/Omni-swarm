#include "loop_net.h"
#include <time.h> 
#include <swarm_msgs/ImageDescriptorHeader_t.hpp>
#include <swarm_msgs/LandmarkDescriptor_t.hpp>

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
    /*
    ROS_INFO("Broadcast Loop Image: size %d", img_des.getEncodedSize());
    if (img_des.getEncodedSize() < 0) {
        ROS_ERROR("WRONG SIZE!!!");
        exit(-1);
    }
    */
    int msg_id = rand() + img_des.timestamp.nsec;
    img_des.msg_id = msg_id;
    sent_message.insert(img_des.msg_id);

    ImageDescriptorHeader_t img_desc_header;
    img_desc_header.timestamp = img_des.timestamp;
    img_desc_header.drone_id = img_des.drone_id;
    img_desc_header.image_desc = img_des.image_desc;
    img_desc_header.pose_drone = img_des.pose_drone;
    img_desc_header.camera_extrinsic = img_des.camera_extrinsic;
    img_desc_header.prevent_adding_db = img_des.prevent_adding_db;
    img_desc_header.msg_id = img_des.msg_id;


    lcm.publish("VIOKF_HEADER", &img_desc_header);

    for (size_t i = 0; i < img_des.landmark_num; i++ ) {
        LandmarkDescriptor_t lm;
        lm.landmark_id = i;
        lm.landmark_2d_norm = img_des.landmarks_2d_norm[i];
        lm.landmark_2d = img_des.landmarks_2d[i];
        lm.landmark_3d = img_des.landmarks_3d[i];
        lm.landmark_flag = img_des.landmarks_flag[i];
        memcpy(lm.feature_descriptor, img_des.feature_descriptor.data() + i *256, 256*sizeof(float));
        
        int msg_id = rand() + img_des.timestamp.nsec;
        sent_message.insert(img_des.msg_id);

        lm.msg_id = msg_id;
        lm.header_id = img_des.msg_id;

        lcm.publish("VIOKF_LANDMARKS", &lm);
    }


    ROS_INFO("Sent Message KEYFRAME with %d landmarks", img_des.landmark_num);
}

void LoopNet::broadcast_loop_connection(LoopConnection & loop_conn) {
    auto _loop_conn = toLCMLoopConnection(loop_conn);
    _loop_conn.msg_id = rand() + loop_conn.ts_a.nsec;

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
    
    ROS_INFO("Received drone %d image from LCM!!!", msg->drone_id);
    this->img_desc_callback(*msg);
}


void LoopNet::on_loop_connection_recevied(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const LoopConnection_t* msg) {

    if (sent_message.find(msg->msg_id) != sent_message.end()) {
        // ROS_INFO("Receive self sent Loop message");
        return;
    }
    ROS_INFO("Received Loop %d->%d from LCM!!!", msg->id_a, msg->id_b);    
    loopconn_callback(*msg);
}