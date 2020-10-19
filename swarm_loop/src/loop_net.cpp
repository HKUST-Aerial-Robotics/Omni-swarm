#include "loop_net.h"
#include <time.h> 

void LoopNet::setup_network(std::string _lcm_uri) {
    if (!lcm.good()) {
        ROS_ERROR("LCM %s failed", _lcm_uri.c_str());
        exit(-1);
    }
    lcm.subscribe("SWARM_LOOP_IMG_DES", &LoopNet::on_img_desc_recevied, this);
    lcm.subscribe("SWARM_LOOP_CONN", &LoopNet::on_loop_connection_recevied, this);
    
    lcm.subscribe("VIOKF_HEADER", &LoopNet::on_img_desc_header_recevied, this);
    lcm.subscribe("VIOKF_LANDMARKS", &LoopNet::on_landmark_recevied, this);

    srand((unsigned)time(NULL)); 
}

void LoopNet::broadcast_img_desc(ImageDescriptor_t & img_des) {
    /*
    if (img_des.getEncodedSize() < 0) {
        ROS_ERROR("WRONG SIZE!!!");
        exit(-1);
    }
    */

    
    int64_t msg_id = rand() + img_des.timestamp.nsec;
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
    img_desc_header.image_desc_size = img_des.image_desc_size;
    img_desc_header.image_desc = img_des.image_desc;
    img_desc_header.feature_num = img_des.landmark_num;
    ROS_INFO("Sent Message VIOHEADER size :%ld", img_desc_header.getEncodedSize());


    lcm.publish("VIOKF_HEADER", &img_desc_header);

    // ROS_INFO("Sending landmarks num: %d, local desc size %d", img_des.landmark_num, img_des.local_descriptors_size);
    for (size_t i = 0; i < img_des.landmark_num; i++ ) {
        LandmarkDescriptor_t lm;
        lm.landmark_id = i;
        lm.landmark_2d_norm = img_des.landmarks_2d_norm[i];
        lm.landmark_2d = img_des.landmarks_2d[i];
        lm.landmark_3d = img_des.landmarks_3d[i];
        lm.landmark_flag = 1;
        lm.drone_id = img_des.drone_id;
        memcpy(lm.feature_descriptor, img_des.feature_descriptor.data() + i *256, 256*sizeof(float));
        
        int64_t msg_id = rand() + img_des.timestamp.nsec;
        sent_message.insert(img_des.msg_id);

        lm.msg_id = msg_id;
        lm.header_id = img_des.msg_id;

        // ROS_INFO("Sending landmark %d, size %d", i, lm.getEncodedSize());
        lcm.publish("VIOKF_LANDMARKS", &lm);
    }

    if (send_img || send_whole_img_desc) {
        if (!send_whole_img_desc) {
            ImageDescriptor_t img_desc_new = img_des;
            img_desc_new.feature_descriptor_size = 0;
            img_desc_new.feature_descriptor.clear();
            ROS_INFO("Sending IMG DES Size %d with %d landmarks.local feature size %d", img_desc_new.getEncodedSize(), img_desc_new.landmark_num, img_desc_new.feature_descriptor_size);
            lcm.publish("SWARM_LOOP_IMG_DES", &img_desc_new);
        } else {
            ROS_INFO("Sending IMG DES Size %d with %d landmarks.local feature size %d", img_des.getEncodedSize(), img_des.landmark_num, img_des.feature_descriptor_size);
            lcm.publish("SWARM_LOOP_IMG_DES", &img_des);
        }
    }
    

    ROS_INFO("Sent Message KEYFRAME %ld with %d landmarks", msg_id, img_des.landmark_num);
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


void LoopNet::on_img_desc_header_recevied(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan, 
    const ImageDescriptorHeader_t* msg) {

    if(msg_blocked(msg->msg_id)) {
        return;
    }

    recv_lock.lock();

    ROS_INFO("Image descriptor from drone (%d) : %ld", msg->drone_id, msg->msg_id);
    update_recv_img_desc_ts(msg->msg_id, true);

    if (receved_msgs.find(msg->msg_id) == receved_msgs.end()) {
        ImageDescriptor_t tmp;
        receved_msgs[msg->msg_id] = tmp; 
        active_receving_msg.insert(msg->msg_id);
    }

    auto & tmp = receved_msgs[msg->msg_id];
    tmp.timestamp = msg->timestamp;
    tmp.drone_id = msg->drone_id;
    tmp.image_desc_size = msg->image_desc_size;
    tmp.image_desc = msg->image_desc;
    tmp.pose_drone = msg->pose_drone;
    tmp.camera_extrinsic = msg->camera_extrinsic;
    tmp.landmark_num = msg->feature_num;
    tmp.msg_id = msg->msg_id;
    tmp.prevent_adding_db = msg->prevent_adding_db;

    recv_lock.unlock();
}

void LoopNet::scan_recv_packets() {
    double tnow = ros::Time::now().toSec();
    std::set<int64_t> finish_recv;
    recv_lock.lock();
    for (auto msg_id : active_receving_msg) {
        if (tnow - msg_header_recv_time[msg_id] > recv_period ||
            receved_msgs[msg_id].landmark_num == receved_msgs[msg_id].landmarks_2d.size()) {
            ROS_INFO("Finish recv msg %ld from drone %d, Feature %ld/%ld", msg_id, receved_msgs[msg_id].drone_id, receved_msgs[msg_id].landmarks_2d.size(), receved_msgs[msg_id].landmark_num);
            receved_msgs[msg_id].landmark_num = receved_msgs[msg_id].landmarks_2d.size();
            finish_recv.insert(msg_id);
        }
    }

    for (auto msg_id : finish_recv) {
        blacklist.insert(msg_id);
        active_receving_msg.erase(msg_id);
    }

    recv_lock.unlock();

    for (auto _id : finish_recv) {
        auto & msg = receved_msgs[_id];
        //Processed recevied message
        if (msg.landmarks_2d.size() > 0) {
            this->img_desc_callback(msg);
        }
        receved_msgs.erase(_id);
    }
}

void LoopNet::on_landmark_recevied(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan, 
    const LandmarkDescriptor_t* msg) {
    if(msg_blocked(msg->header_id)) {
        return;
    }
    recv_lock.lock();
    update_recv_img_desc_ts(msg->header_id, false);
    // ROS_INFO("Landmark %d from drone (%d) : %ld", msg->landmark_id, msg->drone_id, msg->header_id);
    if (receved_msgs.find(msg->header_id) == receved_msgs.end()) {
        ImageDescriptor_t tmp;
        receved_msgs[msg->header_id] = tmp; 
    }

    auto & tmp = receved_msgs[msg->header_id];
    tmp.landmarks_2d_norm.push_back(msg->landmark_2d_norm);
    tmp.landmarks_2d.push_back(msg->landmark_2d);
    tmp.landmarks_3d.push_back(msg->landmark_3d);
    tmp.landmarks_flag.push_back(msg->landmark_flag);
    tmp.feature_descriptor.insert(tmp.feature_descriptor.end(),
        msg->feature_descriptor,
        msg->feature_descriptor+256
    );
    tmp.feature_descriptor_size = tmp.feature_descriptor.size();
    recv_lock.unlock();
    
    scan_recv_packets();
}


void LoopNet::update_recv_img_desc_ts(int64_t id, bool is_header) {
    if(is_header) {
        msg_header_recv_time[id] = ros::Time::now().toSec();
    }
    msg_recv_last_time[id] = ros::Time::now().toSec();
}
