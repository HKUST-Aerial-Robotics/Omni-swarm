#include "ros/ros.h"
#include <swarm_msgs/swarm_frame.h>
#include <swarm_msgs/swarm_detected.h>
#include <nav_msgs/Odometry.h>
#include <mavlink/swarm/mavlink.h>
#include <swarm_msgs/remote_uwb_info.h>
#include <swarm_msgs/data_buffer.h>
#include <swarm_msgs/swarm_fused.h>
#include <swarm_msgs/swarm_fused_relative.h>
#include <swarm_msgs/swarm_remote_command.h>
#include <map>
#include <eigen3/Eigen/Dense>
#include <cstdint>

#define TRANSPORT_LATENCY 0.02f

using namespace swarm_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;

inline Odometry naive_predict(const Odometry &odom_now, double now, bool debug_output = false) {
    static int count = 0;
    Odometry ret = odom_now;
    // double now = ros::Time::now().toSec();
    double t_odom = odom_now.header.stamp.toSec();
    // if (count ++ % 100 == 0)
    if (debug_output) {
        ROS_INFO("Naive1 predict now %f t_odom %f dt %f", now, t_odom, now - t_odom);
    } else {
    }

    ret.pose.pose.position.x += (now - t_odom) * odom_now.twist.twist.linear.x;
    ret.pose.pose.position.y += (now - t_odom) * odom_now.twist.twist.linear.y;
    ret.pose.pose.position.z += (now - t_odom) * odom_now.twist.twist.linear.z;
    ret.header.stamp = ros::Time::now();
    return ret;
}

inline Odometry naive_predict_dt(const Odometry &odom_now, double dt) {
    Odometry ret = odom_now;
    // double now = ros::Time::now().toSec();
    // ROS_INFO("Naive predict now %f t_odom %f dt %f", now, t_odom, now-t_odom);
    ret.pose.pose.position.x += dt * odom_now.twist.twist.linear.x;
    ret.pose.pose.position.y += dt * odom_now.twist.twist.linear.y;
    ret.pose.pose.position.z += dt * odom_now.twist.twist.linear.z;
    ret.header.stamp = ros::Time::now();
    return ret;
}


class LocalProxy {
    ros::NodeHandle &nh;

    ros::Subscriber local_odometry_sub;
    ros::Subscriber swarm_data_sub;
    ros::Subscriber swarm_rel_sub;
    ros::Subscriber swarm_detect_sub;
    ros::Publisher swarm_frame_pub, swarm_frame_nosd_pub;
    ros::Publisher uwb_senddata_pub;

    uint8_t buf[10000] = {0};

    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond quat;


    bool odometry_available = false;
    bool odometry_updated = false;

    nav_msgs::Odometry self_odom;

    int self_id = -1;

    std::vector<swarm_frame> sf_queue;

    void on_swarm_detected(const swarm_msgs::swarm_detected &sd) {
        int i = find_sf_swarm_detected(sd);
        int _detecter_node = sd.self_drone_id;
        if (i > 0) {
            swarm_frame &_sf = sf_queue[i];
            for (int j = 0; j < _sf.node_frames.size(); j++) {
                if (_sf.node_frames[j].id == _detecter_node) {
                    _sf.node_frames[j].detected = sd;
                    return;
                }
            }
            ROS_WARN("Not find id %d in swarmframe", sd.self_drone_id);
        }
    }



    int find_sf_swarm_detected(const swarm_msgs::swarm_detected &sd) {
        for (int i = sf_queue.size() - 1; i >= 0; i--) {
            if ((sf_queue[i].header.stamp - sd.header.stamp).toSec() < 0.02) {
                ROS_INFO("Find sf correspond to sd, dt %3.2fms", (sf_queue[i].header.stamp - sd.header.stamp).toSec());
                return i;
            }
        }
        ROS_WARN("Could not find correspond sf for sd!");
    }

    void on_local_odometry_recv(const nav_msgs::Odometry &odom) {

        // ROS_INFO("Odom recv");
        double t_odom = odom.header.stamp.toSec();
        double now = ros::Time::now().toSec();
        // ROS_INFO("Naive1 predict now %f t_odom %f dt %f", now, t_odom, now-t_odom);
        pos.x() = odom.pose.pose.position.x;
        pos.y() = odom.pose.pose.position.y;
        pos.z() = odom.pose.pose.position.z;

        vel.x() = odom.twist.twist.linear.x;
        vel.y() = odom.twist.twist.linear.y;
        vel.z() = odom.twist.twist.linear.z;


        quat.w() = odom.pose.pose.orientation.w;
        quat.x() = odom.pose.pose.orientation.x;
        quat.y() = odom.pose.pose.orientation.y;
        quat.z() = odom.pose.pose.orientation.z;        

        self_odom = odom;

        odometry_available = true;
        odometry_updated = true;
    }

    bool on_swarm_info_mavlink_msg_recv(mavlink_message_t &msg, nav_msgs::Odometry &odom, std::map<int, float> &_dis) {
        mavlink_node_realtime_info_t swarm_info;

        mavlink_msg_node_realtime_info_decode(&msg, &swarm_info);
        if (!swarm_info.odom_vaild) {
            ROS_INFO("odom not vaild");
            return false;
        }

        odom.pose.pose.position.x = swarm_info.x;
        odom.pose.pose.position.y = swarm_info.y;
        odom.pose.pose.position.z = swarm_info.z;
        
        odom.twist.twist.linear.x = swarm_info.vx/1000.0;
        odom.twist.twist.linear.y = swarm_info.vy/1000.0;
        odom.twist.twist.linear.z = swarm_info.vz/1000.0;

        odom.pose.pose.orientation.w = swarm_info.q0/10000.0;
        odom.pose.pose.orientation.x = swarm_info.q1/10000.0;
        odom.pose.pose.orientation.y = swarm_info.q2/10000.0;
        odom.pose.pose.orientation.z = swarm_info.q3/10000.0;
        
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";
        for (int i = 0; i < 10; i++) {
            if (swarm_info.remote_distance[i] > 0) {
                //When >0, we have it distance for this id
                _dis[i] = swarm_info.remote_distance[i]/1000.0;
            }

        }

    }

    node_detected on_node_detected_msg(int _id, ros::Time stamp, int32_t lps_time_now, mavlink_message_t & msg) {
        //process remode node detected
        //Wait for new inf driver to be used
        mavlink_node_detected_t mdetected;
        node_detected nd;
        mavlink_msg_node_detected_decode(&msg, &mdetected);
        nd.header.stamp = stamp + ros::Duration((mdetected.ts - lps_time_now) / 1000.0);
        nd.self_drone_id = _id;
        nd.remote_drone_id = mdetected.target_id;
        nd.relpose.pose.orientation.w = mdetected.q0/10000.0;
        nd.relpose.pose.orientation.x = mdetected.q1/10000.0;
        nd.relpose.pose.orientation.y = mdetected.q2/10000.0;
        nd.relpose.pose.orientation.z = mdetected.q3/10000.0;
        nd.relpose.pose.position.x = mdetected.x / 1000.0;
        nd.relpose.pose.position.y = mdetected.y / 1000.0;
        nd.relpose.pose.position.z = mdetected.z / 1000.0;


        return nd;
    }

    bool parse_mavlink_data(int _id, ros::Time stamp, int32_t lps_time_now, const std::vector<uint8_t> &buf, nav_msgs::Odometry &odom, std::map<int, float> &_dis, swarm_detected & sd) {
        // mavlink_msg_swa
        mavlink_message_t msg;
        mavlink_status_t status;
        sd.self_drone_id = _id;
        bool ret = false;
        for (uint8_t c : buf) {
            if (mavlink_parse_char(0, c, &msg, &status)) {
                switch(msg.msgid) {
                    case MAVLINK_MSG_ID_NODE_REALTIME_INFO:
                        ret = on_swarm_info_mavlink_msg_recv(msg, odom, _dis);
                        break;
                    case MAVLINK_MSG_ID_NODE_DETECTED:
                        sd.detected_nodes.push_back(on_node_detected_msg(_id, stamp, lps_time_now, msg));
                        sd.header.stamp = sd.detected_nodes.back().header.stamp;
                        break;
                }
            }
        }
        return ret;
    }

    void send_mavlink_message(mavlink_message_t & msg) {
        int len = mavlink_msg_to_send_buffer(buf , &msg);
        data_buffer buffer;
        buffer.data = std::vector<uint8_t>(buf, buf+len);
        uwb_senddata_pub.publish(buffer);
    }

    void send_swarm_mavlink(const float *dis) {

        mavlink_message_t msg;
        double now = ros::Time::now().toSec();

        // auto odom = naive_predict(self_odom, now, false);
        auto pos = self_odom.pose.pose.position;
        auto vel = self_odom.twist.twist.linear;
        auto quat = self_odom.pose.pose.orientation;
        int16_t dis_int[10] = {0};
        for (int i = 0; i< 10; i++) {
            dis_int[i] = (int)(dis[i]*1000);
        }
        mavlink_msg_node_realtime_info_pack(self_id, 0, &msg, odometry_available, pos.x, pos.y, pos.z,
                                            (int)(quat.w*10000), (int)(quat.x*10000), (int)(quat.y*10000), (int)(quat.z*10000),
                                            (int)(vel.x*1000), (int)(vel.y*1000), (int)(vel.z*1000), dis_int);

        send_mavlink_message(msg);
    }

    std::map<int, float> past_self_dis;

    void on_swarm_fused_data_recv(const swarm_fused_relative fused) {

        uint8_t buf[1000] = {0};

        mavlink_message_t msg;
        printf("Fused data recv\n");
        if (self_id < 0)
            return;

        for (int i = 0; i < fused.ids.size(); i++)
        {
            uint8_t _id = fused.ids[i];
            if (_id != self_id)
            {
                printf("Send %d to %d rel\n", self_id, _id);
                mavlink_msg_node_relative_fused_pack(self_id, 0, &msg, _id,
                                                     (int)(fused.relative_drone_position[i].x*1000),
                                                     (int)(fused.relative_drone_position[i].y*1000),
                                                     (int)(fused.relative_drone_position[i].z*1000),
                                                     (int)(fused.relative_drone_yaw[i])*1000);
                send_mavlink_message(msg);
            }
        }
    }

    void process_swarm_frame_queue() {
        //In queue for 5 frame to wait detection
        while (sf_queue.size() > sf_queue_size) {
            auto sf0 = sf_queue[0];
            swarm_frame_pub.publish(sf0);
            ROS_INFO("Queue is len that 5, send to fuse");
            sf_queue.erase(sf_queue.begin());
        }
    }


    bool ParseSwarmInfo(const remote_uwb_info &info, swarm_frame & sf) {
        int drone_num = info.node_ids.size() + 1;
        const std::vector<unsigned int> & ids = info.node_ids;

        std::vector<std::map<int, float>> id_n_distance;
        std::map<int, Odometry> id_odoms;
        std::map<int, bool> vo_available;

        std::vector<unsigned int> available_id;

        available_id.push_back(info.self_id);
        self_id = info.self_id;

        id_n_distance[self_id] = std::map<int, float>();

        for (int i = 0; i < info.node_ids.size(); i++) {
            int _id = info.node_ids[i];

            id_n_distance[self_id][_id] = info.node_dis[i];

            auto s = info.datas[i];
            nav_msgs::Odometry odom;

            std::map<int, float> _dis;
            swarm_detected sd;
            bool ret = parse_mavlink_data(_id, info.header.stamp, info.sys_time, s.data, odom, _dis, sd);

            //Send detected swarm to queue
            if (sd.detected_nodes.size() > 0) {
                on_swarm_detected(sd);
            }

            available_id.push_back(_id);

            if (ret) {
                odom = naive_predict_dt(odom, forward_predict_time);
                vo_available[_id] = true;
                id_n_distance[_id] = _dis;
                id_odoms[_id] = odom;

                if (publish_remote_odom) {
                    if (drone_odom_pubs.find(_id) == drone_odom_pubs.end()) {
                        char name[20] = {0};
                        sprintf(name, "/swarm_drone/odom_%d", _id);

                        drone_odom_pubs[_id] = nh.advertise<Odometry>(name, 1);
                    }
                    drone_odom_pubs[_id].publish(odom);
                }
            }
            else {
                vo_available[_id] = false;
            }
        }

        //sswarm_frame sf;
        sf.self_id = self_id;


        //Because all information we use is from 0.02s ago
        sf.header.stamp = info.header.stamp - ros::Duration(TRANSPORT_LATENCY);
//        sf.header.stamp = info.header.stamp;

        //Switch this to real odom from 0.02 ago
        id_odoms[info.self_id] = naive_predict_dt(self_odom, -TRANSPORT_LATENCY);

        for (int _idx : available_id) {
            node_frame nf;
            nf.id = _idx;
            nf.header.stamp = info.header.stamp;
            nf.vo_available = vo_available[_idx];
            if (nf.vo_available) {
                nf.odometry = id_odoms[_idx];
            }
            for (auto p: id_n_distance[_idx]) {
                nf.dismap_ids.push_back(p.first);
                nf.dismap_dists.push_back(p.second);
            }
        }


        return true;
    }

    void on_remote_nodes_data_recv(const remote_uwb_info &info) {

        //Using last distances, assume cost 0.02 time offset
        swarm_frame sf;

        bool swarm_frame_valid = ParseSwarmInfo(info, sf);

        if (swarm_frame_valid) {
            sf_queue.push_back(sf);
        }

        float self_dis[100] = {0};
        std::map<int, float> self_dis_vec;

        for (int i = 0; i < info.node_ids.size(); i++) {
            int _id = info.node_ids[i];

            self_dis[_id] = info.node_dis[i];
            self_dis_vec[_id] = info.node_dis[i];
        }

        for (int i = 0; i < 10; i++) {
            self_dis[i] = -1;
        }


        send_swarm_mavlink(self_dis);


        if (odometry_available && odometry_updated) {
            odometry_updated = false;
            swarm_frame_nosd_pub.publish(sf);
            process_swarm_frame_queue();
        }
    }



    std::map<int, ros::Publisher> drone_odom_pubs;
    double forward_predict_time = 0.08;
    bool publish_remote_odom = false;
    int sf_queue_size = 5;




public:
    LocalProxy(ros::NodeHandle &_nh) :
            nh(_nh) {
        ROS_INFO("Start SWARM Drone Proxy");

        nh.param<int>("sf_queue_size", sf_queue_size, 5);
        nh.param<bool>("publish_remote_odom", publish_remote_odom, false);
        nh.param<double>("forward_predict_time", forward_predict_time , 0.02);
        // read /vins_estimator/odometry and send to uwb by mavlink
        local_odometry_sub = nh.subscribe("/vins_estimator/imu_propagate", 1, &LocalProxy::on_local_odometry_recv, this,
                                          ros::TransportHints().tcpNoDelay());
        swarm_data_sub = nh.subscribe("/uwb_node/remote_nodes", 1, &LocalProxy::on_remote_nodes_data_recv, this,
                                      ros::TransportHints().tcpNoDelay());
        swarm_rel_sub = nh.subscribe("/swarm_drones/swarm_drone_fused_relative", 1,
                                     &LocalProxy::on_swarm_fused_data_recv, this, ros::TransportHints().tcpNoDelay());

        swarm_detect_sub = nh.subscribe("/swarm_detection/swarm_detected", 10, &LocalProxy::on_swarm_detected, this,
                                        ros::TransportHints().tcpNoDelay());
        swarm_frame_pub = nh.advertise<swarm_frame>("/swarm_drones/swarm_frame", 10);

        swarm_frame_nosd_pub = nh.advertise<swarm_frame>("/swarm_drones/swarm_frame_without_detection", 10);
        swarm_frame_pub = nh.advertise<swarm_frame>("/swarm_drones/swarm_frame", 10);

        uwb_senddata_pub = nh.advertise<data_buffer>("/uwb_node/send_broadcast_data", 10);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization_proxy");
    ros::NodeHandle nh("localization_proxy");
    new LocalProxy(nh);
    // ros::AsyncSpinner spinner(4);
    // spinner.start();
    ros::spin();
    ros::waitForShutdown();
}