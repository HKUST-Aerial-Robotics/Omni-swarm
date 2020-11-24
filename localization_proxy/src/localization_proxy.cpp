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
//#define DISABLE_DETECTION_6D
#define YAW_UNAVAIL 32767

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
    ros::Subscriber swarm_fused_sub;
    ros::Subscriber swarm_detect_sub;
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
            // ROS_INFO("Find sf correspond to sd, dt %3.2fms", min_time*1000);
            return best;
        }
        return -1;
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
        // self_odom.header.stamp = ros::Time::now();
        self_odoms.push_back(odom);

        if (self_odoms.size() > 1000) {
            self_odoms.erase(self_odoms.begin());
        }

        odometry_available = true;
        odometry_updated = true;

        // ROS_INFO("ODOM OK");
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
            ROS_WARN_THROTTLE(0.1, "odom not vaild %d", _id);
            return false;
        }
        // ROS_INFO("x %d y %d z %d")
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


        if (dt < 100) {
            // ROS_INFO_THROTTLE_NAMED(1.0, "PROXY_RECV", "ID %d NR RECV %d now %d DT %d POS %3.2f %3.2f %3.2f VEL %3.2f %3.2f %3.2f", 
            // _id, node_realtime_info.lps_time, tn, dt, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z);
        } else {
            // ROS_WARN_THROTTLE_NAMED(0.1, "PROXY_RECV", "ID %d NodeRealtime RECV %d now %d DT %d", _id, node_realtime_info.lps_time, tn, dt);
        }


        return true;
    }

    node_detected_xyzyaw on_node_detected_msg(int _id, mavlink_message_t &msg) {
        //process remode node detected
        //Wait for new inf driver to be used
        mavlink_node_detected_t mdetected;
        node_detected_xyzyaw nd;
        mavlink_msg_node_detected_decode(&msg, &mdetected);
        nd.header.stamp = LPS2ROSTIME(mdetected.lps_time);
        int32_t tn = ROSTIME2LPS(ros::Time::now());
        int32_t dt = tn - mdetected.lps_time;
        if (dt < 100) {
            ROS_INFO_THROTTLE(1.0, "ND RECV %d now %d DT %d", mdetected.lps_time, tn, tn - mdetected.lps_time);
            // ROS_INFO("ND RECV %d now %d DT %d", mdetected.lps_time, tn, tn - mdetected.lps_time);
        } else {
            ROS_WARN_THROTTLE(1.0, "NodeDetected RECV %d now %d DT %d", mdetected.lps_time, tn, tn - mdetected.lps_time);
        }

        nd.self_drone_id = _id;
        nd.remote_drone_id = mdetected.target_id;
        nd.dpos.x = mdetected.x / 1000.0;
        nd.dpos.y = mdetected.y / 1000.0;
        nd.dpos.z = mdetected.z / 1000.0;
        if (mdetected.inv_dep != 0) {
            nd.inv_dep = mdetected.inv_dep / 10000.0;
            nd.enable_scale = true;                    
        } else {
            nd.enable_scale = false;                    
        }

        //Update with swarm detection should be same
        nd.dpos_cov.x = 0.02;
        nd.dpos_cov.y = 0.01;   
        nd.dpos_cov.z = 0.01;

        nd.dyaw_cov = 10/57.3;
        nd.is_yaw_valid = false;                    
        return nd;
    }

    void send_node_detected(const swarm_msgs::node_detected_xyzyaw & nd) {
        mavlink_message_t msg;
        int32_t ts = ROSTIME2LPS(nd.header.stamp);
        // ROS_INFO("SEND ND ts %d now %d", ts, ROSTIME2LPS(ros::Time::now()));
        int inv_dep = 0;
        if (nd.enable_scale) {
            inv_dep = nd.inv_dep * 10000.0;
            if (inv_dep > 65535) {
                inv_dep = 65535;
            }
        }

        mavlink_msg_node_detected_pack(self_id, 0, &msg, ts, nd.remote_drone_id, 
            (int)(nd.dpos.x*1000),
            (int)(nd.dpos.y*1000),
            (int)(nd.dpos.z*1000),
            (int)(nd.probaility*10000),
            inv_dep);
        
        send_mavlink_message(msg, true);
    }


    nav_msgs::Odometry find_nearest_selfpose(ros::Time stamp) {
        if (self_odoms.size() == 0) {
            nav_msgs::Odometry odom;
            odom.pose.pose.position.x = 0;
            odom.pose.pose.position.y = 0;
            odom.pose.pose.position.z = 0;

            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = 0;
            odom.pose.pose.orientation.w = 1;

            return odom;
        }
        /*
        ros::Time s_last = self_odoms[self_odoms.size() - 1].header.stamp;
        double dt = (s_last - stamp).toSec();
        int c = dt*400;
        if (self_odoms.size() - c <= 0) {
            return self_odoms[0];
        }

        if (c < 0) {
            return self_odoms[self_odoms.size() - 1];
        }

        ROS_INFO("DT %fms C %d PTR %ld final dt %fms", dt*1000, c, self_odoms.size() - c, (self_odoms[self_odoms.size() - c - 1].header.stamp - stamp).toSec() * 1000);
        */

        for (int ptr = self_odoms.size() - 1; ptr >= 1; ptr --) {
            double dt1 = (self_odoms[ptr].header.stamp - stamp).toSec();
            double dt2 = (self_odoms[ptr - 1].header.stamp - stamp).toSec();
            if (dt1 < 0) {
                return self_odoms[ptr];
            }
            if (dt1 > 0 && dt2 <= 0) {
                if (fabs(dt1) < fabs(dt2)) {
                    //ROS_INFO("final dt %fms", dt1 * 1000);
                    return self_odoms[ptr];
                } else {
                    //ROS_INFO("final dt %fms", dt2 * 1000);
                    return self_odoms[ptr-1];
                }
            }
        }

        return self_odoms[0];
    }

    node_detected_xyzyaw ndxyzyaw_from_nd(const node_detected & _nd, const nav_msgs::Odometry _self_odom) {
        assert(odometry_available && "Using ndxyz from nd must have vo!");
        node_detected_xyzyaw _nd_xyzyaw;
        _nd_xyzyaw.header.stamp = _nd.header.stamp;
        _nd_xyzyaw.self_drone_id = _nd.self_drone_id;
        _nd_xyzyaw.remote_drone_id = _nd.remote_drone_id;
        if (_nd_xyzyaw.self_drone_id < 0) {
            _nd_xyzyaw.self_drone_id = self_id;
        }
        //For xyz yaw relpose, we first need compute the pose of target
        Swarm::Pose relpose(_nd.relpose.pose);

        //TODO: Here we will have timestamp align problem... Need to fix that
        Swarm::Pose selfpose(_self_odom.pose.pose);

        Swarm::Pose remotepose = selfpose.to_isometry() *relpose.to_isometry();
        Eigen::Vector3d dpos = remotepose.pos() - selfpose.pos();
        dpos = Eigen::AngleAxisd(- selfpose.yaw(), Vector3d::UnitZ()) * dpos;
        _nd_xyzyaw.dpos.x = dpos.x();
        _nd_xyzyaw.dpos.y = dpos.y();
        _nd_xyzyaw.dpos.z = dpos.z();

    
        _nd_xyzyaw.dyaw = remotepose.yaw() - selfpose.yaw();

        _nd_xyzyaw.dpos_cov.x = _nd.relpose.covariance[0];
        _nd_xyzyaw.dpos_cov.y = _nd.relpose.covariance[6+1];
        _nd_xyzyaw.dpos_cov.z = _nd.relpose.covariance[2*6+2];
        _nd_xyzyaw.dyaw_cov = _nd.relpose.covariance[5*6+5];
        _nd_xyzyaw.is_yaw_valid = _nd.is_yaw_valid;  
        // _nd_xyzyaw.enable_scale = _nd.enable_scale;                  

        return _nd_xyzyaw;
    }

    std::vector<node_detected_xyzyaw> convert_sd_to_nd_xyzyaw(const swarm_msgs::swarm_detected & sd) {
        std::vector<node_detected_xyzyaw> ret;
        nav_msgs::Odometry odom = find_nearest_selfpose(sd.header.stamp);
        if (!odometry_available) {
            ROS_WARN("Odometry unavailable, can't compute node detected xyzyaw");
            return ret;
        }

        for (const node_detected & _nd: sd.detected_nodes) {
            if (_nd.remote_drone_id != self_id) {
                ret.push_back(ndxyzyaw_from_nd(_nd, odom));
            }
        }

        return ret;
    }

    void on_swarm_detected(const swarm_msgs::swarm_detected & sd) {
        ROS_INFO("SD size %ld", sd.detected_nodes_xyz_yaw.size());
        if (sd.detected_nodes_xyz_yaw.size() == 0) {
            return;
        }

        auto node_xyzyaws = sd.detected_nodes_xyz_yaw;
        
        for (node_detected_xyzyaw nd : node_xyzyaws) {
            send_node_detected(nd);
        }

        int i = find_sf_swarm_detected(sd.header.stamp);
        int sd_self_id = sd.self_drone_id;
        if (sd_self_id < 0) {
            sd_self_id = self_id;
        }
        if (i < 0) {
            ROS_WARN("Can't find id %d in swarmframe", sd_self_id);
            return;
        }
        swarm_frame &_sf = sf_queue[i];


        // ROS_INFO("SF node size %ld", _sf.node_frames.size());
        for (int j = 0; j < _sf.node_frames.size(); j++) {
            // ROS_INFO("NF id %d", _sf.node_frames[j].id);
            if (_sf.node_frames[j].id == sd_self_id) {
                _sf.node_frames[j].detected = node_xyzyaws;
                ROS_INFO("SF BUF %d got detection", j);
                break;
            }
        }

    }

    //Eul is roll pitch yaw
    void add_odom_dis_to_sf(swarm_frame & sf, int _id, geometry_msgs::Point pos, Eigen::Vector3d eul, geometry_msgs::Point vel, ros::Time _time, std::map<int, float> & _dis) {
        for (node_frame & nf : sf.node_frames) {
            if (nf.id == _id && !nf.vo_available) {
                //Easy to deal with this, add only first time
                nf.position = pos;
                nf.velocity = vel;
                nf.yaw = eul.z();
                nf.pitch = eul.y();
                nf.roll = eul.x();
                nf.vo_available = true;
                nf.header.stamp = _time;
                
                for (auto it : _dis) {
                    nf.dismap_ids.push_back(it.first);
                    nf.dismap_dists.push_back(it.second);
                }
                return;
            }
        }
    }

    void add_node_detected_to_sf(swarm_frame & sf, const node_detected_xyzyaw & nd) {
        for (node_frame & nf : sf.node_frames) {
            if (nf.id == nd.self_drone_id) {
                nf.detected.push_back(nd);
                return;
            }
        }
    }


    void parse_node_realtime_info(mavlink_message_t & msg, int _id) {
        Odometry odom;
        std::map<int, float> _dis;
        ros::Time ts;
        geometry_msgs::Point pos, vel;
        Eigen::Vector3d eul;
        bool ret = on_node_realtime_info_mavlink_msg_recv(msg, _id, ts, pos, eul, vel, _dis);
        if (ret) {
            int s_index = find_sf_swarm_detected(ts);
            if (s_index >= 0) {
                ROS_INFO_THROTTLE(1.0, "Appending ODOM DIS TS %5.1f sf to frame %d/%ld", (ts - this->tsstart).toSec()*1000, s_index, sf_queue.size());
                add_odom_dis_to_sf(sf_queue[s_index], _id, pos, eul, vel, ts, _dis);
            }
        }
    }

    void parse_node_detected(mavlink_message_t & msg, int _id) {
        node_detected_xyzyaw nd = on_node_detected_msg(_id, msg);
        ros::Time ts = nd.header.stamp;
        int s_index = find_sf_swarm_detected(ts);
        ROS_INFO_THROTTLE(1.0, "Appending ND %dby%d TS %5.1f(%5.1f) sf to frame %d/%ld", 
            nd.remote_drone_id,
            nd.self_drone_id,
            (ts - this->tsstart).toSec()*1000, 
            (ros::Time::now() - this->tsstart).toSec()*1000, 
            s_index, sf_queue.size());

        if (s_index >= 0) {
            add_node_detected_to_sf(sf_queue[s_index], nd);
        }
    }

    void parse_mavlink_data(incoming_broadcast_data income_data) {
        // ROS_INFO("incoming data ts %d", income_data.lps_time);
        int _id = income_data.remote_id;
        if (_id == self_id) {
            ROS_WARN("Receive self message; Return");
            return;
        }
        auto buf = income_data.data;
        mavlink_message_t msg;
        mavlink_status_t status;

        for (uint8_t c : buf) {
            //Use different to prevent invaild parse
            int ret = mavlink_parse_char(_id, c, &msg, &status);
            if (ret) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_NODE_REALTIME_INFO: {
                        parse_node_realtime_info(msg, _id);
                        break;
                    }

                    case MAVLINK_MSG_ID_NODE_DETECTED: {
#ifndef DISABLE_DETECTION_6D
                        parse_node_detected(msg, _id);
#endif
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
        // ROS_INFO("Msg size %d", len);

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
            // ROS_INFO("dis i %d: %d", i, dis_int[i]);
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
                uint8_t buf[1000] = {0};
                send_fused_basecoor_count ++;
                int _index = send_fused_basecoor_count % basecoor.ids.size();
                /*
                if (basecoor.ids[_index] == self_id) {
                    if (basecoor.ids.size() == 1) {
                        return;
                    } else {
                        send_fused_basecoor_count ++;
                        _index = send_fused_basecoor_count % basecoor.ids.size();
                    }
                }*/

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
            uint8_t buf[1000] = {0};

            mavlink_message_t msg;
            // printf("Fused data recv\n");
            if (self_id < 0)
                return;

            int32_t ts = ROSTIME2LPS(fused.header.stamp);
            for (int i = 0; i < fused.ids.size(); i++) {
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
                uint8_t buf[1000] = {0};
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
//            ROS_INFO("Queue is len that 5, send to fuse");
            sf_queue.erase(sf_queue.begin());
        }
    }


    node_frame * find_lastest_nf_with_vo_in_queue(int _id) {
        //Find where this nf has vo available in our sldwin
        for (int ptr = sf_queue.size() - 1; ptr >= 0; ptr --) {
            swarm_frame & _sf = sf_queue[ptr];
            for (int k = 0; k < _sf.node_frames.size(); k++) {
                node_frame & _nf = _sf.node_frames[k];
                if (_nf.id == _id) {
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
        ROS_INFO_THROTTLE_NAMED(1.0, "PROXY_FOR_PREIDCT", "Predict NF %d DT %3.2fms DX %3.2f %3.2f %3.2f mm with vel %3.2f %3.2f %3.2f mm/s", _nf.id, dt*1000,
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
                ROS_WARN_THROTTLE(1.0, "Node %d can't find in queue %ld", _id, sf_queue.size());
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
        // ROS_INFO("SF TS %f", (sf.header.stamp - this->tsstart).toSec()*1000);
        //        sf.header.stamp = info.header.stamp;

        //Switch this to real odom from 0.02 ago

        if (_force_id < 0) {
            node_frame self_nf;
            self_nf.id = self_id;
            self_nf.header.stamp = self_odom.header.stamp;
            self_nf.vo_available = odometry_available;
            Eigen::Quaterniond q;
            q.w() = self_odom.pose.pose.orientation.w;
            q.x() = self_odom.pose.pose.orientation.x;
            q.y() = self_odom.pose.pose.orientation.y;
            q.z() = self_odom.pose.pose.orientation.z;
            Eigen::Vector3d rpy = quat2eulers(q);
            self_nf.yaw = rpy.z();
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
        // ROS_INFO("SF TS %f", (sf.header.stamp - this->tsstart).toSec()*1000);
        //        sf.header.stamp = info.header.stamp;

        //Switch this to real odom from 0.02 ago

        if (_force_id < 0) {
            node_frame self_nf;
            self_nf.id = self_id;
            self_nf.header.stamp = self_odom.header.stamp;
            self_nf.vo_available = odometry_available;
            Eigen::Quaterniond q;
            q.w() = self_odom.pose.pose.orientation.w;
            q.x() = self_odom.pose.pose.orientation.x;
            q.y() = self_odom.pose.pose.orientation.y;
            q.z() = self_odom.pose.pose.orientation.z;
            Eigen::Vector3d rpy = quat2eulers(q);
            self_nf.yaw = rpy.z();
            self_nf.position.x = self_odom.pose.pose.position.x;
            self_nf.position.y = self_odom.pose.pose.position.y;
            self_nf.position.z = self_odom.pose.pose.position.z;
            self_nf.velocity.x = self_odom.twist.twist.linear.x;
            self_nf.velocity.y = self_odom.twist.twist.linear.y;
            self_nf.velocity.z = self_odom.twist.twist.linear.z;

            for (int i = 0; i < info.node_ids.size(); i++) {
                int _idx = info.node_ids[i];
                if (info.active[i] && _idx!=self_id) {
                    self_nf.dismap_ids.push_back(_idx);
                    self_nf.dismap_dists.push_back(info.node_dis[i]);
                }
            }

            sf.node_frames.push_back(self_nf);
        }
        all_nodes.insert(self_id);
        for (int i = 0; i< info.node_ids.size(); i++) {
            int _idx = info.node_ids[i];
            if (info.active[i] && _idx!=self_id) {
                node_frame nf;
                nf.id = _idx;
                all_nodes.insert(_idx);

                nf.header.stamp = sf.header.stamp;
                nf.vo_available = false;
                sf.node_frames.push_back(nf);
            }

        }

        return sf;
    }

    void on_uwb_distance_measurement(const remote_uwb_info &info) {
        ROS_INFO_THROTTLE(1.0, "Recv RTnode LPS  time %d now %d", info.sys_time, ROSTIME2LPS(ros::Time::now()));
        //TODO: Deal with rssi here

        //Using last distances, assume cost 0.02 time offset
        swarm_frame sf = create_swarm_frame_from_uwb(info);
//        ROS_INFO("push sf %d", info.sys_time);
        sf_queue.push_back(sf);

        float self_dis[100] = {0};

        for (int i = 0; i < 10; i++) {
            self_dis[i] = -1;
        }

        for (int i = 0; i < info.node_ids.size(); i++) {
            int _id = info.node_ids[i];
            if (_id < MAX_DRONE_SIZE) {
                self_dis[_id] = info.node_dis[i];
            } else {
                ROS_WARN_THROTTLE(1.0, "Node %d:%f is out of max drone size, not sending", _id, info.node_dis[i]);
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
    bool _transmission_wifi = false;
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
        ROS_INFO("Start SWARM Drone Proxy");
        // bigger than 3 is ok
        nh.param<int>("sf_queue_size", sf_queue_max_size, 10);
        nh.param<int>("force_id", _force_id, -1); //Use a force id to publish the swarm frame messages, which means you can direct use gcs to compute same thing
        nh.param<int>("self_id", self_id, 0); 

        nh.param<bool>("publish_remote_odom", publish_remote_odom, false);

        nh.param<bool>("enable_uwb", _enable_uwb, true);
        nh.param<bool>("transmission_wifi", _transmission_wifi, false);
        
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
#ifndef DISABLE_DETECTION_6D
        swarm_detect_sub = nh.subscribe("/swarm_detection/swarm_detected", 10, &LocalProxy::on_swarm_detected, this,
                                        ros::TransportHints().tcpNoDelay());
#endif


        swarm_frame_nosd_pub = nh.advertise<swarm_frame>("/swarm_drones/swarm_frame_predict", 1);
        swarm_frame_pub = nh.advertise<swarm_frame>("/swarm_drones/swarm_frame", 10);
        
        based_sub = nh.subscribe("/swarm_drones/swarm_drone_basecoor", 1, &LocalProxy::on_swarm_fused_basecoor_recv, this, ros::TransportHints().tcpNoDelay());

        if (_enable_uwb) {
            uwb_timeref_sub = nh.subscribe("/uwb_node/time_ref", 1, &LocalProxy::on_uwb_timeref, this, ros::TransportHints().tcpNoDelay());
            swarm_data_sub = nh.subscribe("/uwb_node/remote_nodes", 1, &LocalProxy::on_uwb_distance_measurement, this,
                                      ros::TransportHints().tcpNoDelay());
            if (!_transmission_wifi) {
                uwb_senddata_pub = nh.advertise<data_buffer>("/uwb_node/send_broadcast_data", 1);
                uwb_incoming_sub = nh.subscribe("/uwb_node/incoming_broadcast_data", 10, &LocalProxy::parse_mavlink_data, this, ros::TransportHints().tcpNoDelay());
            }
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
    ros::init(argc, argv, "localization_proxy");
    ros::NodeHandle nh("localization_proxy");
    new LocalProxy(nh);
    //ros::MultiThreadedSpinner spinner(4);
    //spinner.spin();
    
    ros::spin();
    ros::waitForShutdown();
}
