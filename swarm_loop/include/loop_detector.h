#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <swarm_msgs/ImageDescriptor.h>
#include <swarm_msgs/LoopConnection.h>
#include <swarm_msgs/ImageDescriptor_t.hpp>
#include "loop_defines.h"
#include <loop_cam.h>
#include <functional>
#include <swarm_msgs/Pose.h>

#ifdef USE_DEEPNET
#include <faiss/IndexFlat.h>
#else
#include <DBoW3/DBoW3.h>
#endif

using namespace swarm_msgs;

class LoopDetector {

protected:
#ifdef USE_DEEPNET
    faiss::IndexFlatIP index;
#else
    DBoW3::Vocabulary voc;
    DBoW3::Database db;
#endif
    std::map<unsigned int, ImageDescriptor_t> id2imgdes;
    std::map<unsigned int, cv::Mat> id2cvimg;
    std::vector<cv::Scalar> colors;
    bool compute_loop(const ImageDescriptor_t & new_img_desc, const ImageDescriptor_t & old_img_desc, cv::Mat img_new, cv::Mat img_old, LoopConnection & ret, bool init_mode=false);
    bool compute_relative_pose(cv::Mat & img_new_small, cv::Mat & img_old_small, const std::vector<cv::Point2f> & nowPtsSmall, 
        const std::vector<cv::Point2f> now_norm_2d,
        const std::vector<cv::Point3f> now_3d,
        Swarm::Pose old_extrinsic,
        Swarm::Pose drone_pose_now,
        Swarm::Pose & DP_old_to_new,
        bool init_mode,
        bool use_orb_matching);

    int add_to_database(const ImageDescriptor_t & new_img_desc);
    int query_from_database(const ImageDescriptor_t & new_img_desc, int max_index, bool init_mode=false);

    void find_correspoding_pts(cv::Mat img1, cv::Mat img2, std::vector<cv::Point2f> Pts, std::vector<cv::Point2f> &tracked, 
            std::vector<unsigned char> & status, bool init_mode, bool visualize = false);

    std::set<int> success_loop_nodes;
    std::set<int> all_nodes;

public:
    std::function<void(LoopConnection &)> on_loop_cb;
    int self_id = -1;
#ifdef USE_DEEPNET
    LoopDetector();
#else
    LoopDetector(const std::string & voc_path);
#endif
    void on_image_recv(const ImageDescriptor_t & img_des, cv::Mat img = cv::Mat());
    void on_loop_connection(LoopConnection & loop_conn);
    LoopCam * loop_cam = nullptr;
    bool enable_visualize = true;
    cv::Mat decode_image(const ImageDescriptor_t & _img_desc);

    int database_size() const;

};
