#include "ImageDescriptor_t.hpp"
#include <swarm_msgs/ImageDescriptor.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <opencv2/opencv.hpp>
#include <swarm_msgs/LoopConnection.h>
#include "LoopConnection_t.hpp"
#include <swarm_msgs/Pose.h>

inline swarm_msgs::ImageDescriptor toROSMsg(const ImageDescriptor_t & _img) {
    swarm_msgs::ImageDescriptor img_des;
    return img_des;
}


inline Pose_t fromROSPose(const geometry_msgs::Pose & pose) {
    Pose_t t;
    t.orientation[0] = pose.orientation.x;
    t.orientation[1] = pose.orientation.y;
    t.orientation[2] = pose.orientation.z;
    t.orientation[3] = pose.orientation.w;

    t.position[0] = pose.position.x;
    t.position[1] = pose.position.y;
    t.position[2] = pose.position.z;
    return t;
}

inline geometry_msgs::Pose toROSPose(const Pose_t & t) {
    geometry_msgs::Pose pose;
    pose.orientation.x = t.orientation[0];
    pose.orientation.y = t.orientation[1];
    pose.orientation.z = t.orientation[2];
    pose.orientation.w = t.orientation[3];

    pose.position.x = t.position[0];
    pose.position.y = t.position[1];
    pose.position.z = t.position[2];
    return pose;
}



inline void ROSPoints2LCM(const std::vector<geometry_msgs::Point32> & src, std::vector<Point2d_t> & dst) {
    for (auto pt : src) {
        Point2d_t pt2;
        pt2.x = pt.x;
        pt2.y = pt.y;
        dst.push_back(pt2);
    }
}

inline void ROSPoints2LCM(const std::vector<geometry_msgs::Point32> & src, std::vector<Point3d_t> & dst) {
    for (auto pt : src) {
        Point3d_t pt3;
        pt3.x = pt.x;
        pt3.y = pt.y;
        pt3.z = pt.z;
        dst.push_back(pt3);
    }
}


inline cv::Mat cvfeatureFromByte(uint8_t*data, int feature_num, int feature_len = 32) {
    cv::Mat mat(feature_num, feature_len, CV_8UC1);
    memcpy(mat.data, data, feature_num*feature_len*sizeof(uint8_t));
    return mat;
}


inline cv::Point2f toCV(Point2d_t a) {
    cv::Point2f pt;
    pt.x = a.x;
    pt.y = a.y;
    return pt;
}

inline cv::Point3f toCV(Point3d_t a) {
    cv::Point3f pt;
    pt.x = a.x;
    pt.y = a.y;
    pt.z = a.z;
    return pt;
}


inline std::vector<cv::Point2f> toCV(std::vector<Point2d_t> arr) {
    std::vector<cv::Point2f> _arr;
    for (auto a : arr) {
        _arr.push_back(toCV(a));
    }
    return _arr;
}


inline std::vector<cv::Point2f> toCV(Point2d_t * arr, int len) {
    std::vector<cv::Point2f> _arr;
    for (int i = 0; i < len; i++) {
        auto a = arr[i];
        _arr.push_back(toCV(a));
    }
    return _arr;
}


inline ros::Time toROSTime(Time_t _time) {
    return ros::Time(_time.sec, _time.nsec);
}

inline Time_t toLCMTime(ros::Time _time) {
    Time_t t;
    t.sec = _time.sec;
    t.nsec = _time.nsec;
    return t;
}


inline swarm_msgs::LoopConnection toROSLoopConnection(const LoopConnection_t & loop_con) {
    swarm_msgs::LoopConnection loop_conn;
    loop_conn.ts_a =  toROSTime(loop_con.ts_a);
    loop_conn.ts_b =  toROSTime(loop_con.ts_b);

    loop_conn.id_a = loop_con.id_a;
    loop_conn.id_b = loop_con.id_b;

    loop_conn.dpos.x = loop_con.dpos.x;
    loop_conn.dpos.y = loop_con.dpos.y;
    loop_conn.dpos.z = loop_con.dpos.z;
    loop_conn.dyaw = loop_con.dyaw;

    loop_conn.self_pose_a = toROSPose(loop_con.self_pose_a);
    loop_conn.self_pose_b = toROSPose(loop_con.self_pose_b);

    return loop_conn;
}


inline LoopConnection_t toLCMLoopConnection(const swarm_msgs::LoopConnection & loop_con) {
    LoopConnection_t loop_conn;
    loop_conn.ts_a =  toLCMTime(loop_con.ts_a);
    loop_conn.ts_b =  toLCMTime(loop_con.ts_b);

    loop_conn.id_a = loop_con.id_a;
    loop_conn.id_b = loop_con.id_b;

    loop_conn.dpos.x = loop_con.dpos.x;
    loop_conn.dpos.y = loop_con.dpos.y;
    loop_conn.dpos.z = loop_con.dpos.z;


    loop_conn.self_pose_a = fromROSPose(loop_con.self_pose_a);
    loop_conn.self_pose_b = fromROSPose(loop_con.self_pose_b);

    return loop_conn;
}
