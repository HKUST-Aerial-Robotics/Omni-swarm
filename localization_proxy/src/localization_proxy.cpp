#include "ros/ros.h"
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <map>
#include <mavlink/swarm/mavlink.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/TimeReference.h>
#include <swarm_msgs/swarm_frame.h>
#include <swarm_msgs/swarm_fused.h>
#include <swarm_msgs/swarm_fused_relative.h>
#include <swarm_msgs/swarm_drone_basecoor.h>
#include <swarm_msgs/swarm_remote_command.h>
#include <swarm_msgs/swarm_detected.h>
#include <swarm_msgs/node_detected_xyzyaw.h>
#include <swarm_msgs/Pose.h>

#include <swarmcomm_msgs/incoming_broadcast_data.h>
#include <swarmcomm_msgs/data_buffer.h>
#include <swarmcomm_msgs/remote_uwb_info.h>
#include <geometry_msgs/Point.h>
#include <map>


using namespace swarm_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace  swarmcomm_msgs;
// using namespace Swarm;

#define MAX_DRONE_SIZE 10
#define INVAILD_DISTANCE 65535
#define YAW_UNAVAIL 32767

#define SWARM_DETECTION_ON_FRAME


#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
    backward::SignalHandling sh;
}

inline double float_constrain(double v, double min, double max)
{
    if (v < min) {
        return min;
    }
    if (v > max) {
        return max;
    }
    return v;
}


geometry_msgs::Quaternion Yaw2ROSQuat(double yaw) {
    geometry_msgs::Quaternion _q;
    Eigen::Quaterniond quat = (Eigen::Quaterniond) (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    _q.w = quat.w();
    _q.x = quat.x();
    _q.y = quat.y();
    _q.z = quat.z();
    return _q;
}

inline Odometry naive_predict(const Odometry &odom_now, double now, bool debug_output = false) {
    Odometry ret = odom_now;
    double t_odom = odom_now.header.stamp.toSec();
    if (debug_output) {
        ROS_INFO("[LOCAL_PROXY] Naive predict now %f t_odom %f dt %f", now, t_odom, now - t_odom);
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
    // ROS_INFO("[LOCAL_PROXY] Naive predict now %f t_odom %f dt %f", now, t_odom, now-t_odom);
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
    ros::Subscriber swarm_fused_sub;
    ros::Subscriber swarm_detect_sub;
    ros::Publisher swarm_detect_pub;
    ros::Publisher swarm_frame_pub, swarm_frame_nosd_pub;
    ros::Publisher uwb_senddata_pub;
    ros::Subscriber uwb_timeref_sub;
    ros::Subscriber uwb_incoming_sub;

    uint8_t buf[10000] = {0};

    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond quat;

    bool odometry_available = false;
    bool odometry_updated = false;

    nav_msgs::Odometry self_odom;
    std::vector<nav_msgs::Odometry> self_odoms;

    int self_id = -1;

    std::vector<swarm_frame> sf_queue;

    

    int find_sf_swarm_detected(ros::Time ts) {
        double min_time = 0.015;
        int best = -1;
        for (int i = sf_queue.size() - 1; i >= 0; i--) {
            if (fabs((sf_queue[i].header.stamp - ts).toSec()) < min_time) {
                best = i;
                min_time = (sf_queue[i].header.stamp - ts).toSec();
            }
        }

        if (best >=0) {
            // ROS_INFO("[LOCAL_PROXY] Find sf correspond to sd, dt %3.2fms", min_time*1000);
            return best;
        }
        return -1;
    }

    void on_local_odometry_recv(const nav_msgs::Odometry &odom) {

        // ROS_INFO("[LOCAL_PROXY] Odom recv");
        // double t_odom = odom.header.stamp.toSec();
        // double now = ros::Time::now().toSec();
        // ROS_INFO("[LOCAL_PROXY] Naive1 predict now %f t_odom %f dt %f", now, t_odom, now-t_odom);
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
        // self_odom.header.stamp = ros::Time::now();
        self_odoms.push_back(odom);

        if (self_odoms.size() > 1000) {
            self_odoms.erase(self_odoms.begin());
        }

        odometry_available = true;
        odometry_updated = true;

        // ROS_INFO("[LOCAL_PROXY] ODOM OK");
    }

    node_detected on_node_detected_msg(int _id, mavlink_message_t &msg) {
        //process remode node detected
        //Wait for new inf driver to be used
        mavlink_node_detected_t mdetected;
        node_detected nd;
        mavlink_msg_node_detected_decode(&msg, &mdetected);
        nd.header.stamp = LPS2ROSTIME(mdetected.lps_time);
        int32_t tn = ROSTIME2LPS(ros::Time::now());
        int32_t dt = tn - mdetected.lps_time;
        ROS_INFO("[LOCAL_PROXY] Recv node_detected ID %d  XYZ %lf %lf %lf YAW %lfdeg", mdetected.id, mdetected.rel_x, mdetected.rel_y, mdetected.rel_z, mdetected.rel_yaw*57.3);
        
        nd.self_drone_id = _id;
        nd.id = mdetected.id;
        nd.remote_drone_id = mdetected.target_id;
        Swarm::Pose pose(Eigen::Vector3d(mdetected.rel_x, mdetected.rel_y, mdetected.rel_z), mdetected.rel_yaw);
        nd.relative_pose.pose = pose.to_ros_pose();
        nd.is_yaw_valid = true;   


        Eigen::Map<Eigen::Matrix<double,6,6,RowMajor>> cov(nd.relative_pose.covariance.data());
        cov.setZero();
        cov(0, 0) = mdetected.cov_x;
        cov(1, 1) = mdetected.cov_y;
        cov(2, 2) = mdetected.cov_z;
        cov(3, 3) = mdetected.cov_yaw;
        cov(4, 4) = mdetected.cov_yaw;
        cov(5, 5) = mdetected.cov_yaw;
        return nd;
    }

    void send_node_detected(const swarm_msgs::node_detected & nd) {
        mavlink_message_t msg;
        int32_t ts = ROSTIME2LPS(nd.header.stamp);
        int inv_dep = 0;
        
        auto quat = nd.relative_pose.pose.orientation;
        Eigen::Quaterniond _q(quat.w, quat.x, quat.y, quat.z);
        Eigen::Vector3d eulers = quat2eulers(_q);
        auto pos = nd.relative_pose.pose.position;
        const Eigen::Map<const Eigen::Matrix<double,6,6,RowMajor>> cov(nd.relative_pose.covariance.data());
        ROS_INFO("[LOCAL_PROXY] Send node_detected ID %d XYZ %lf %lf %lf YAW %lfdeg quat (WXYZ) %lf %lf %lf %lf", nd.id, pos.x, pos.y, pos.z, eulers(2)*57.3, quat.w, quat.x, quat.y, quat.z);

        mavlink_msg_node_detected_pack(self_id, 0, &msg, ts, nd.id, nd.remote_drone_id, 
            (float)(pos.x),
            (float)(pos.y),
            (float)(pos.z),
            (float)(eulers(2)),
            cov(0, 0),
            cov(1, 1),
            cov(2, 2),
            cov(5, 5));
        
        send_mavlink_message(msg, true);
    }


    void parse_node_detected(mavlink_message_t & msg, int _id) {
        ROS_INFO("Node detected recv");
        auto nd = on_node_detected_msg(_id, msg);
        swarm_detect_pub.publish(nd);
    }

    void on_swarm_detected(const swarm_msgs::swarm_detected & sd) {
        if (sd.detected_nodes_xyz_yaw.size() + sd.detected_nodes.size() == 0) {
            return;
        }

        ROS_WARN("[LOCAL_PROXY] Broadcast detecteds");
        for (auto nd : sd.detected_nodes) {
            send_node_detected(nd);
        }
    }

    //Eul is roll pitch yaw
    void add_odom_dis_to_sf(swarm_frame & sf, int _id, geometry_msgs::Point pos, Eigen::Vector3d eul, geometry_msgs::Point vel, ros::Time _time, std::map<int, float> & _dis) {
        for (node_frame & nf : sf.node_frames) {
            if (nf.drone_id == _id && !nf.vo_available) {
                //Easy to deal with this, add only first time
                nf.position = pos;
                nf.velocity = vel;
                nf.yaw = eul.z();
                nf.pitch = eul.y();
                nf.roll = eul.x();
                nf.vo_available = true;
                nf.header.stamp = _time;
                auto quat = eulers2quat(eul);
                nf.quat.x = quat.x();
                nf.quat.y = quat.y();
                nf.quat.z = quat.z();
                nf.quat.w = quat.w();
                
                for (auto it : _dis) {
                    nf.dismap_ids.push_back(it.first);
                    nf.dismap_dists.push_back(it.second);
                }
                return;
            }
        }
    }

    //EUL is roll pitch yaw
    bool on_node_realtime_info_mavlink_msg_recv(mavlink_message_t &msg, int _id, ros::Time & ts, Point & pos, Eigen::Vector3d & eul, Point & vel, std::map<int, float> &_dis) {
        mavlink_node_realtime_info_t node_realtime_info;
        mavlink_msg_node_realtime_info_decode(&msg, &node_realtime_info);

        ts = LPS2ROSTIME(node_realtime_info.lps_time);
        //This odom is quat only and don't have yaw
        int32_t tn = ROSTIME2LPS(ros::Time::now());
        int32_t dt = tn - node_realtime_info.lps_time;
        
        if (!node_realtime_info.odom_vaild) {
            ROS_INFO_THROTTLE(1.0, "[PROXY_RECV] odom not vaild of drone %d", _id);
            return false;
        }
        // ROS_INFO("[LOCAL_PROXY] x %d y %d z %d")
        // pos.x = node_realtime_info.x/1000.0;
        // pos.y = node_realtime_info.y/1000.0;
        // pos.z = node_realtime_info.z/1000.0;
        pos.x = node_realtime_info.x;
        pos.y = node_realtime_info.y;
        pos.z = node_realtime_info.z;
        vel.x = node_realtime_info.vx / 100.0;
        vel.y = node_realtime_info.vy / 100.0;
        vel.z = node_realtime_info.vz / 100.0;

        eul.z() = node_realtime_info.yaw / 1000.0;
        eul.y() = node_realtime_info.pitch / 1000.0;
        eul.x() = node_realtime_info.roll / 1000.0;

        for (int i = 0; i < MAX_DRONE_SIZE; i++) {
            if (node_realtime_info.remote_distance[i] > 0) {
                //When >0, we have it distance for this id
                if (node_realtime_info.remote_distance[i] == INVAILD_DISTANCE) {
                    // _dis[i] = -1;
                } else {
                    _dis[i] = node_realtime_info.remote_distance[i] / 1000.0;
                }
            }
        }
        
        
        // if (dt < 100) {
                // ROS_INFO("[LOCAL_PROXY] ID %d T %d Pose now %d DT %d POS %3.2f %3.2f %3.2f VEL %3.2f %3.2f %3.2f", 
                // _id, node_realtime_info.lps_time, tn, dt, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z);
        // } else {
        //     ROS_WARN("[LOCAL_PROXY] ID %d Pose RECV %d now %d DT %d", _id, node_realtime_info.lps_time, tn, dt);
        // }
        return true;
    }

    void parse_node_realtime_info(mavlink_message_t & msg, int _id) {
        // ROS_INFO("[LOCAL_PROXY] parse_node_realtime_info");
        Odometry odom;
        std::map<int, float> _dis;
        ros::Time ts;
        geometry_msgs::Point pos, vel;
        Eigen::Vector3d eul;
        bool ret = on_node_realtime_info_mavlink_msg_recv(msg, _id, ts, pos, eul, vel, _dis);
        if (ret) {
            int s_index = find_sf_swarm_detected(ts);
            if (s_index >= 0) {
                ROS_INFO_THROTTLE(1.0, "[LOCAL_PROXY] Appending ODOM DIS TS %5.1f sf to frame %d/%ld", (ts - this->tsstart).toSec()*1000, s_index, sf_queue.size());
                add_odom_dis_to_sf(sf_queue[s_index], _id, pos, eul, vel, ts, _dis);
            } else {
                ROS_WARN_THROTTLE(1.0, "[LOCAL_PROXY] add_odom_dis_to_sf ID:%d failed queue_size %d", _id, sf_queue.size());
                if (sf_queue.size() >= 2) {
                    // ROS_WARN_THROTTLE(1.0, "[LOCAL_PROXY] add_odom_dis_to_sf ID:%d failed (sf_queue.front() - ts) %4.3f (sf_queue.back() - ts) %4.3f size: %d",
                    ROS_WARN("[LOCAL_PROXY] add_odom_dis_to_sf ID:%d failed (sf_queue.front() - ts) %4.3f (sf_queue.back() - ts) %4.3f size: %d",
                        _id, 
                        (sf_queue.front().header.stamp - ts).toSec(),
                        (sf_queue.back().header.stamp - ts).toSec(),
                        sf_queue.size()
                    );
                }
            }
        }
    }

    void parse_mavlink_data(incoming_broadcast_data income_data) {
        // ROS_INFO("[LOCAL_PROXY] incoming data ts %d", income_data.lps_time);
        int _id = income_data.remote_id;
        if (_id == self_id) {
            ROS_WARN("[LOCAL_PROXY] Receive self message %d/%d; Return", _id, self_id);
            return;
        }
        auto buf = income_data.data;
        mavlink_message_t msg;
        mavlink_status_t status;

        for (uint8_t c : buf) {
            //Use different to prevent invaild parse
            int ret = mavlink_parse_char(0, c, &msg, &status);
            if (ret) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_NODE_REALTIME_INFO: {
                        parse_node_realtime_info(msg, _id);
                        break;
                    }

                    case MAVLINK_MSG_ID_NODE_DETECTED: {
                        parse_node_detected(msg, _id);
                        break;
                    }
                }
            } else {
                if (ret == MAVLINK_FRAMING_BAD_CRC) {
                    ROS_WARN("Mavlink parse error");   
                }
            }
        }
    }

    void send_mavlink_message(mavlink_message_t &msg, bool send_by_wifi=false) {
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        data_buffer buffer;
        // ROS_INFO("[LOCAL_PROXY] Msg size %d", len);

        buffer.data = std::vector<uint8_t>(buf, buf + len);

        if (send_by_wifi) {
            buffer.send_method = 2;
        }
        uwb_senddata_pub.publish(buffer);
    }

    void send_self_odometry_and_distance(int32_t ts, const float *dis) {

        mavlink_message_t msg;

        // auto odom = naive_predict(self_odom, now, false);
        auto pos = self_odom.pose.pose.position;
        auto vel = self_odom.twist.twist.linear;
        auto quat = self_odom.pose.pose.orientation;
        uint16_t dis_int[MAX_DRONE_SIZE] = {0};
        for (int i = 0; i < MAX_DRONE_SIZE; i++) {
            if (dis[i] < 0) {
                dis_int[i] = INVAILD_DISTANCE;
            } else {
                dis_int[i] = (int)(dis[i] * 1000);
            }
            // ROS_INFO("[LOCAL_PROXY] dis i %d: %d", i, dis_int[i]);
        }
        Eigen::Quaterniond _q(quat.w, quat.x, quat.y, quat.z);
        Eigen::Vector3d eulers = quat2eulers(_q);

        mavlink_msg_node_realtime_info_pack(self_id, 0, &msg, ts, odometry_available, pos.x, pos.y, pos.z, 
            int(vel.x*100), int(vel.y*100), int(vel.z*100), int(eulers.x()*1000), int(eulers.y()*1000), int(eulers.z()*1000), dis_int);

        send_mavlink_message(msg, true);
    }

    std::map<int, float> past_self_dis;

    ros::Time last_send_fused = ros::Time::now();
    ros::Time last_send_fused_base = ros::Time::now();
    ros::Time last_send_rel_fused = ros::Time::now();


    double send_fused_freq = 1.0;
    double send_rel_fused_freq = 1.0;
    double send_fused_basecoor_freq = 40.0;
    int send_fused_basecoor_count = 0;


    void on_swarm_fused_basecoor_recv(const swarm_drone_basecoor & basecoor) {
        if (send_fused_basecoor_freq < 0.1) {
            return;
        }


        if ((ros::Time::now() - last_send_fused_base).toSec() > 1.0/send_fused_basecoor_freq) {
                send_fused_basecoor_count ++;
                int _index = send_fused_basecoor_count % basecoor.ids.size();
                mavlink_message_t msg;
                // printf("Fused data recv\n");
                if (self_id < 0)
                    return;

               
                uint8_t _id = basecoor.ids[_index];

                int32_t ts = ROSTIME2LPS(basecoor.header.stamp);

                mavlink_msg_node_based_fused_pack(self_id, 0, &msg, ts, _id,
                                                        (int)(basecoor.drone_basecoor[_index].x * 1000),
                                                        (int)(basecoor.drone_basecoor[_index].y * 1000),
                                                        (int)(basecoor.drone_basecoor[_index].z * 1000),
                                                        (int)(basecoor.drone_baseyaw[_index] * 1000),
                                                        (int)(basecoor.position_cov[_index].x * 1000),
                                                        (int)(basecoor.position_cov[_index].y * 1000),
                                                        (int)(basecoor.position_cov[_index].z * 1000),
                                                        (int)(float_constrain(basecoor.yaw_cov[_index], 0, M_PI*M_PI) * 1000));
                send_mavlink_message(msg, true);
                
                last_send_fused_base = ros::Time::now();
        }
    }

    void on_swarm_fused_relative_recv(const swarm_fused_relative & fused) {

        if (send_rel_fused_freq < 0.1) {
            return;
        }
        
        if ((ros::Time::now() - last_send_rel_fused).toSec() > 1.0/send_rel_fused_freq) {
            // uint8_t buf[1000] = {0};

            mavlink_message_t msg;
            // printf("Fused data recv\n");
            if (self_id < 0)
                return;

            int32_t ts = ROSTIME2LPS(fused.header.stamp);
            for (unsigned int i = 0; i < fused.ids.size(); i++) {
                uint8_t _id = fused.ids[i];
                mavlink_msg_node_relative_fused_pack(self_id, 0, &msg, ts, _id,
                                                    (int)(fused.relative_drone_position[i].x * 1000),
                                                    (int)(fused.relative_drone_position[i].y * 1000),
                                                    (int)(fused.relative_drone_position[i].z * 1000),                                    
                                                    (int)(fused.relative_drone_yaw[i] * 1000),
                                                    (int)(fused.position_cov[i].x * 1000),
                                                    (int)(fused.position_cov[i].y * 1000),
                                                    (int)(fused.position_cov[i].z * 1000),
                                                    (int)(float_constrain(fused.yaw_cov[i], 0, M_PI*M_PI) * 1000));
                
                send_mavlink_message(msg, true);
            }

            last_send_rel_fused = ros::Time::now();
        }

    }

    int send_fused_count = 0;

    void on_swarm_fused_recv(const swarm_fused & fused) {

        if (send_fused_freq < 0.1) {
            return;
        }

        if ((ros::Time::now() - last_send_fused).toSec() > 1.0/send_fused_freq) {
                // uint8_t buf[1000] = {0};
                send_fused_count ++;
                int _index = send_fused_count % fused.ids.size();
                if (fused.ids[_index] == self_id) {
                    if (fused.ids.size() == 1) {
                        return;
                    } else {
                        send_fused_count ++;
                        _index = send_fused_count % fused.ids.size();
                    }
                }

                mavlink_message_t msg;
                // printf("Fused data recv\n");
                if (self_id < 0)
                    return;

               
                uint8_t _id = fused.ids[_index];

                int32_t ts = ROSTIME2LPS(fused.header.stamp);

                if (_id != self_id) {
                    mavlink_msg_node_local_fused_pack(self_id, 0, &msg, ts, _id,
                                                        (int)(fused.local_drone_position[_index].x * 1000),
                                                        (int)(fused.local_drone_position[_index].y * 1000),
                                                        (int)(fused.local_drone_position[_index].z * 1000),
                                                        (int)(fused.local_drone_yaw[_index] * 1000),
                                                        (int)(fused.position_cov[_index].x * 1000),
                                                        (int)(fused.position_cov[_index].y * 1000),
                                                        (int)(fused.position_cov[_index].z * 1000),
                                                        (int)(float_constrain(fused.yaw_cov[_index], 0, M_PI*M_PI) * 1000));
                    send_mavlink_message(msg, true);
                }
                
                last_send_fused = ros::Time::now();
        }
    }

    void process_swarm_frame_queue() {
        //In queue for 5 frame to wait detection
        while (sf_queue.size() > sf_queue_max_size) {
            auto sf0 = sf_queue[0];
            swarm_frame_pub.publish(sf0);
//            ROS_INFO("[LOCAL_PROXY] Queue is len that 5, send to fuse");
            sf_queue.erase(sf_queue.begin());
        }
    }


    node_frame * find_lastest_nf_with_vo_in_queue(int _id) {
        //Find where this nf has vo available in our sldwin
        for (int ptr = sf_queue.size() - 1; ptr >= 0; ptr --) {
            swarm_frame & _sf = sf_queue[ptr];
            for (unsigned int k = 0; k < _sf.node_frames.size(); k++) {
                node_frame & _nf = _sf.node_frames[k];
                if (_nf.drone_id == _id) {
                    if (_nf.vo_available){
                        return &_nf;
                    } else {
                    //jump this loop
                        break;
                    }
                }
            }

        }
        return nullptr;
    }

    node_frame predict_nf(const node_frame & _nf, const ros::Time t) const {
        node_frame nf = _nf;
        double dt = (t - _nf.header.stamp).toSec();
        nf.position.x += _nf.velocity.x * dt;
        nf.position.y += _nf.velocity.y * dt;
        nf.position.z += _nf.velocity.z * dt;
        ROS_INFO_THROTTLE_NAMED(1.0, "[LOCAL_PROXY] PROXY_FOR_PREIDCT", "Predict NF %d DT %3.2fms DX %3.2f %3.2f %3.2f mm with vel %3.2f %3.2f %3.2f mm/s", _nf.drone_id, dt*1000,
            _nf.velocity.x * dt*1000, _nf.velocity.y * dt*1000, _nf.velocity.z * dt*1000,
            _nf.velocity.x*1000,_nf.velocity.y*1000, _nf.velocity.z*1000
            );

        return nf;
    }

    void send_predicted_swarm_frame() {
        swarm_frame sf;
        //Will predict till to vo stamp now
        ros::Time tnow = self_odom.header.stamp;
        if (_force_id > 0 || ! odometry_available) {
            tnow = ros::Time::now(); // If use force id, than directly use now time
        }

        sf.header.stamp = tnow;
        sf.self_id = self_id;

        for (int _id : all_nodes) {
            node_frame * _nf = find_lastest_nf_with_vo_in_queue(_id);
            if (_nf != nullptr) {
                node_frame nf = predict_nf(*_nf, tnow);
                sf.node_frames.push_back(nf);
                // ROS_INFO_THROTTLE_NAMED(1.0, "PROXY_FOR_PREIDCT", "Predict NF %d DT %3.2fms", _id, (tnow - _nf->header.stamp).toSec()*1000 );
            } else {
                ROS_WARN_THROTTLE(1.0, "[LOCAL_PROXY] Node %d can't find in queue %ld", _id, sf_queue.size());
            }

        }

        swarm_frame_nosd_pub.publish(sf);
    }

    std::set<int> all_nodes;


    swarm_frame create_swarm_frame_from_self_odom() {
        swarm_frame sf;

        std::map<int, Odometry> id_odoms;
        std::map<int, bool> vo_available;

        //Because all information we use is from 0.02s ago
        sf.header.stamp = self_odom.header.stamp;
        // ROS_INFO("[LOCAL_PROXY] SF TS %f", (sf.header.stamp - this->tsstart).toSec()*1000);
        //        sf.header.stamp = info.header.stamp;

        //Switch this to real odom from 0.02 ago

        if (_force_id < 0) {
            node_frame self_nf;
            self_nf.drone_id = self_id;
            self_nf.header.stamp = self_odom.header.stamp;
            self_nf.vo_available = odometry_available;
            Eigen::Quaterniond q;
            q.w() = self_odom.pose.pose.orientation.w;
            q.x() = self_odom.pose.pose.orientation.x;
            q.y() = self_odom.pose.pose.orientation.y;
            q.z() = self_odom.pose.pose.orientation.z;
            self_nf.quat = self_odom.pose.pose.orientation;
            Eigen::Vector3d rpy = quat2eulers(q);
            self_nf.yaw = rpy.z();
            self_nf.pitch = rpy.y();
            self_nf.roll = rpy.x();
            self_nf.position.x = self_odom.pose.pose.position.x;
            self_nf.position.y = self_odom.pose.pose.position.y;
            self_nf.position.z = self_odom.pose.pose.position.z;
            self_nf.velocity.x = self_odom.twist.twist.linear.x;
            self_nf.velocity.y = self_odom.twist.twist.linear.y;
            self_nf.velocity.z = self_odom.twist.twist.linear.z;
            sf.node_frames.push_back(self_nf);
        }

        all_nodes.insert(self_id);

        return sf;
    }

    swarm_frame create_swarm_frame_from_uwb(const remote_uwb_info &info) {
        swarm_frame sf;

        int drone_num = info.node_ids.size() + 1;
        const std::vector<unsigned int> &ids = info.node_ids;

        std::map<int, Odometry> id_odoms;
        std::map<int, bool> vo_available;

        std::vector<unsigned int> available_id = info.node_ids;

        //sswarm_frame sf;
        if (_force_id < 0) {
            sf.self_id = self_id = info.self_id;
        } else {
            self_id = sf.self_id = _force_id;
        }

        //Because all information we use is from 0.02s ago
        sf.header.stamp = LPS2ROSTIME(info.sys_time);
        // ROS_INFO("[LOCAL_PROXY] SF TS %f", (sf.header.stamp - this->tsstart).toSec()*1000);
        //        sf.header.stamp = info.header.stamp;

        //Switch this to real odom from 0.02 ago

        if (_force_id < 0) {
            node_frame self_nf;
            self_nf.drone_id = self_id;
            self_nf.header.stamp = self_odom.header.stamp;
            self_nf.vo_available = odometry_available;
            Eigen::Quaterniond q;
            q.w() = self_odom.pose.pose.orientation.w;
            q.x() = self_odom.pose.pose.orientation.x;
            q.y() = self_odom.pose.pose.orientation.y;
            q.z() = self_odom.pose.pose.orientation.z;
            Eigen::Vector3d rpy = quat2eulers(q);
            self_nf.yaw = rpy.z();
            self_nf.quat = self_odom.pose.pose.orientation;
            self_nf.position.x = self_odom.pose.pose.position.x;
            self_nf.position.y = self_odom.pose.pose.position.y;
            self_nf.position.z = self_odom.pose.pose.position.z;
            self_nf.velocity.x = self_odom.twist.twist.linear.x;
            self_nf.velocity.y = self_odom.twist.twist.linear.y;
            self_nf.velocity.z = self_odom.twist.twist.linear.z;

            for (unsigned int i = 0; i < info.node_ids.size(); i++) {
                int _idx = info.node_ids[i];
                if (info.active[i] && _idx!=self_id) {
                    self_nf.dismap_ids.push_back(_idx);
                    self_nf.dismap_dists.push_back(info.node_dis[i]);
                }
            }

            sf.node_frames.push_back(self_nf);
        }
        all_nodes.insert(self_id);
        for (unsigned int i = 0; i< info.node_ids.size(); i++) {
            int _idx = info.node_ids[i];
            if (info.active[i] && _idx!=self_id) {
                node_frame nf;
                nf.drone_id = _idx;
                all_nodes.insert(_idx);

                nf.header.stamp = sf.header.stamp;
                nf.vo_available = false;
                sf.node_frames.push_back(nf);
            }

        }

        return sf;
    }

    void on_uwb_distance_measurement(remote_uwb_info info) {
        ROS_INFO_THROTTLE(1.0, "[LOCAL_PROXY] Recv RTnode LPS  time %d now %d", info.sys_time, ROSTIME2LPS(ros::Time::now()));
        // ROS_INFO("[LOCAL_PROXY] Recv RTnode LPS T %d stamp %d now %d", info.sys_time, ROSTIME2LPS(info.header.stamp), ROSTIME2LPS(ros::Time::now()));
        //TODO: Deal with rssi here

        //Using last distances, assume cost 0.02 time offset
        swarm_frame sf = create_swarm_frame_from_uwb(info);
//        ROS_INFO("[LOCAL_PROXY] push sf %d", info.sys_time);
        sf_queue.push_back(sf);

        float self_dis[100] = {0};

        for (int i = 0; i < 10; i++) {
            self_dis[i] = -1;
        }

        for (unsigned int i = 0; i < info.node_ids.size(); i++) {
            int _id = info.node_ids[i];
            if (_id < MAX_DRONE_SIZE) {
                self_dis[_id] = info.node_dis[i];
            } else {
                ROS_WARN_THROTTLE(1.0, "[LOCAL_PROXY] Node %d:%f is out of max drone size, not sending", _id, info.node_dis[i]);
            }
        }

        send_self_odometry_and_distance(info.sys_time, self_dis);

        odometry_updated = false;

        send_predicted_swarm_frame();
        process_swarm_frame_queue();
    }


    void predict_swarm_frame_callback(const ros::TimerEvent & e) {
        swarm_frame sf = create_swarm_frame_from_self_odom();

        sf_queue.push_back(sf);

        float self_dis[100] = {0};

        for (int i = 0; i < 10; i++) {
            self_dis[i] = -1;
        }

        send_self_odometry_and_distance( ROSTIME2LPS(sf.header.stamp), self_dis);

        odometry_updated = false;

        send_predicted_swarm_frame();
        process_swarm_frame_queue();
    }

    std::map<int, ros::Publisher> drone_odom_pubs;
    bool publish_remote_odom = false;
    int sf_queue_max_size = 10;
    bool _enable_uwb = true;
    sensor_msgs::TimeReference uwb_time_ref;

    int _force_id = -1;
    void on_uwb_timeref(const sensor_msgs::TimeReference &ref) {
        uwb_time_ref = ref;
    }       

    ros::Time LPS2ROSTIME(const int32_t &lps_time) {
        ros::Time base = uwb_time_ref.header.stamp - ros::Duration(uwb_time_ref.time_ref.toSec());
        return base + ros::Duration(lps_time / 1000.0);
    }

    int32_t ROSTIME2LPS(ros::Time ros_time) {
        double lps_t_s = (ros_time - uwb_time_ref.header.stamp).toSec() + uwb_time_ref.time_ref.toSec();
        return (int32_t)(lps_t_s * 1000);
    }
    ros::Time tsstart = ros::Time::now();

    ros::Subscriber based_sub;
    ros::Timer keyframe_callback_timer;

public:
    LocalProxy(ros::NodeHandle &_nh) : nh(_nh) {
        ROS_INFO("[LOCAL_PROXY] Start SWARM Drone Proxy. Mavlink channels %d", MAVLINK_COMM_NUM_BUFFERS);
        // bigger than 3 is ok
        nh.param<int>("sf_queue_max_size", sf_queue_max_size, 10);
        ROS_INFO("[LOCAL_PROXY] sf_queue_max_size %d", sf_queue_max_size);
        nh.param<int>("force_id", _force_id, -1); //Use a force id to publish the swarm frame messages, which means you can direct use gcs to compute same thing
        nh.param<int>("self_id", self_id, 0); 

        nh.param<bool>("publish_remote_odom", publish_remote_odom, false);

        nh.param<bool>("enable_uwb", _enable_uwb, true);
        
        nh.param<double>("send_fused_freq", send_fused_freq, 30.0);
        nh.param<double>("send_rel_fused_freq", send_rel_fused_freq, 0.0);
        nh.param<double>("send_fused_basecoor_freq", send_fused_basecoor_freq, 30.0);
        // read /vins_estimator/odometry and send to uwb by mavlink
        local_odometry_sub = nh.subscribe("/vins_estimator/imu_propagate", 10, &LocalProxy::on_local_odometry_recv, this,
                                          ros::TransportHints().tcpNoDelay());
       
        swarm_rel_sub = nh.subscribe("/swarm_drones/swarm_drone_fused_relative", 1,
                                     &LocalProxy::on_swarm_fused_relative_recv, this, ros::TransportHints().tcpNoDelay());
        
        swarm_fused_sub = nh.subscribe("/swarm_drones/swarm_drone_fused", 1,
                                     &LocalProxy::on_swarm_fused_recv, this, ros::TransportHints().tcpNoDelay());

        swarm_detect_sub = nh.subscribe("/swarm_detection/swarm_detected_raw", 10, &LocalProxy::on_swarm_detected, this,
                                        ros::TransportHints().tcpNoDelay());

        swarm_frame_pub = nh.advertise<swarm_frame>("/swarm_drones/swarm_frame", 10);
        swarm_frame_nosd_pub = nh.advertise<swarm_frame>("/swarm_drones/swarm_frame_predict", 10);

        swarm_detect_pub = nh.advertise<node_detected>("/swarm_drones/node_detected_6d", 10);
        
        based_sub = nh.subscribe("/swarm_drones/swarm_drone_basecoor", 1, &LocalProxy::on_swarm_fused_basecoor_recv, this, ros::TransportHints().tcpNoDelay());

        if (_enable_uwb) {
            uwb_timeref_sub = nh.subscribe("/uwb_node/time_ref", 1, &LocalProxy::on_uwb_timeref, this, ros::TransportHints().tcpNoDelay());
            swarm_data_sub = nh.subscribe("/uwb_node/remote_nodes", 1, &LocalProxy::on_uwb_distance_measurement, this,
                                      ros::TransportHints().tcpNoDelay());
            uwb_senddata_pub = nh.advertise<data_buffer>("/uwb_node/send_broadcast_data", 1);
            uwb_incoming_sub = nh.subscribe("/uwb_node/incoming_broadcast_data", 10, &LocalProxy::parse_mavlink_data, this, ros::TransportHints().tcpNoDelay());
        } else {
            //Init LCM transmission here
            //
            keyframe_callback_timer =  nh.createTimer(ros::Duration(0.01), &LocalProxy::predict_swarm_frame_callback, this);
            
            uwb_senddata_pub = nh.advertise<data_buffer>("/uwb_node/send_broadcast_data", 1);
            uwb_incoming_sub = nh.subscribe("/uwb_node/incoming_broadcast_data", 10, &LocalProxy::parse_mavlink_data, this, ros::TransportHints().tcpNoDelay());
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "LOCAL_PROXY");
    ros::NodeHandle nh("LOCAL_PROXY");
    new LocalProxy(nh);
    //ros::MultiThreadedSpinner spinner(4);
    //spinner.spin();
    
    ros::spin();
    ros::waitForShutdown();
}
