#include "ImageDescriptor_t.hpp"
#include <swarm_msgs/ImageDescriptor.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <opencv2/opencv.hpp>
#include <swarm_msgs/LoopConnection.h>
#include "LoopConnection_t.hpp"
#include <swarm_msgs/Pose.h>




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

inline void LCMPoints2ROS(const std::vector<Point2d_t> & src, std::vector<geometry_msgs::Point32> & dst) {
    for (auto pt : src) {
        geometry_msgs::Point32 pt2;
        pt2.x = pt.x;
        pt2.y = pt.y;
        dst.push_back(pt2);
    }
}

inline void LCMPoints2ROS(const std::vector<Point3d_t> & src, std::vector<geometry_msgs::Point32> & dst) {
    for (auto pt : src) {
        geometry_msgs::Point32 pt2;
        pt2.x = pt.x;
        pt2.y = pt.y;
        pt2.z = pt.z;
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

inline std::vector<cv::Point3f> toCV(Point3d_t * arr, int len) {
    std::vector<cv::Point3f> _arr;
    for (int i = 0; i < len; i++) {
        auto a = arr[i];
        _arr.push_back(toCV(a));
    }
    return _arr;
}

inline std::vector<cv::Point3f> toCV(std::vector<Point3d_t> arr) {
    std::vector<cv::Point3f> _arr;
    for (auto a : arr) {
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


inline std::vector<cv::KeyPoint> to_keypoints(const std::vector<cv::Point2f> & pts) {
    std::vector<cv::KeyPoint> kps;
    for (auto pt : pts) {
        cv::KeyPoint kp;
        kp.pt = pt;
        kps.push_back(kp);
    }
    return kps;
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
    loop_conn.dyaw = loop_con.dyaw;


    loop_conn.self_pose_a = fromROSPose(loop_con.self_pose_a);
    loop_conn.self_pose_b = fromROSPose(loop_con.self_pose_b);

    return loop_conn;
}


inline swarm_msgs::ImageDescriptor toROSImageDescriptor(const ImageDescriptor_t & _img) {
    swarm_msgs::ImageDescriptor img_desc;
    img_desc.header.stamp = toROSTime(_img.timestamp);
    img_desc.drone_id = _img.drone_id;
    img_desc.feature_descriptor = _img.feature_descriptor;
    img_desc.pose_drone = toROSPose(_img.pose_drone);
    img_desc.camera_extrinsic = toROSPose(_img.camera_extrinsic);
    
    LCMPoints2ROS(_img.landmarks_2d_norm, img_desc.landmarks_2d_norm);
    LCMPoints2ROS(_img.landmarks_2d, img_desc.landmarks_2d);
    LCMPoints2ROS(_img.landmarks_3d, img_desc.landmarks_3d);

    // geometry_msgs/Point32[] all_features_2d
    // LCMPoints2ROS(_img.keyfeature_point_2d_norm, img_desc.keyfeature_point_2d_norm);
    // LCMPoints2ROS(_img.keyfeature_point_3d, img_desc.keyfeature_point_3d);
   
    img_desc.image_desc = _img.image_desc;
    img_desc.image_width = _img.image_width;
    img_desc.image_height = _img.image_height;
    img_desc.image = _img.image;
    img_desc.prevent_adding_db = _img.prevent_adding_db;
    img_desc.landmarks_flag = _img.landmarks_flag;
    return img_desc;
}

inline ImageDescriptor_t toLCMImageDescriptor(const swarm_msgs::ImageDescriptor & img_desc) {
    ImageDescriptor_t _img;
    _img.timestamp = toLCMTime(img_desc.header.stamp);
    _img.drone_id = img_desc.drone_id;
    _img.feature_descriptor = img_desc.feature_descriptor;
    _img.feature_descriptor_size = img_desc.feature_descriptor.size();

    _img.pose_drone = fromROSPose(img_desc.pose_drone);
    _img.camera_extrinsic = fromROSPose(img_desc.camera_extrinsic);
    
    ROSPoints2LCM(img_desc.landmarks_2d_norm, _img.landmarks_2d_norm);
    ROSPoints2LCM(img_desc.landmarks_2d, _img.landmarks_2d);
    ROSPoints2LCM(img_desc.landmarks_3d, _img.landmarks_3d);
    _img.landmark_num = _img.landmarks_2d_norm.size();

    _img.image_desc_size = 0;
    // geometry_msgs/Point32[] all_features_2d
    // LCMPoints2ROS(_img.keyfeature_point_2d_norm, img_desc.keyfeature_point_2d_norm);
    // LCMPoints2ROS(_img.keyfeature_point_3d, img_desc.keyfeature_point_3d);
   
    _img.image_desc = img_desc.image_desc;
    _img.image_width = img_desc.image_width;
    _img.image_height = img_desc.image_height;
    _img.image = img_desc.image;
    _img.image_size = img_desc.image.size();
    _img.prevent_adding_db = img_desc.prevent_adding_db;
    _img.landmarks_flag = img_desc.landmarks_flag;

    return _img;
}
