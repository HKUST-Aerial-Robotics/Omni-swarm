#include <loop_cam.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <chrono> 
using namespace std::chrono; 

LoopCam::LoopCam(const std::string & camera_config_path, const std::string & BRIEF_PATTERN_FILE) {
    camodocal::CameraFactory cam_factory;
    cam = cam_factory.generateCameraFromYamlFile(camera_config_path);
}

void LoopCam::on_camera_message(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    image_queue.push_back(ptr->image);
    image_ts_queue.push_back(msg->header.stamp.toSec());
    cam_count ++;
}

std::pair<ImageDescriptor_t, cv::Mat>  LoopCam::on_keyframe_message(const vins::VIOKeyframe& msg) {
    ROS_INFO("Received new keyframe. with %ld landmarks...", msg.feature_points_2d_uv.size());
    
    cv::Mat img = pop_image_ts(msg.header.stamp);
    ImageDescriptor_t ides;
    ides.landmark_num = 0;
    if (img.empty()) {
        ROS_INFO("No Image; Exiting;");
        cv::Mat _img;
        return std::pair<ImageDescriptor_t, cv::Mat> (ides, _img);
    }
    auto start = high_resolution_clock::now();
    ides = feature_detect(img);
    std::cout << "FeatureDetect Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 << "ms" << std::endl;

    ides.timestamp = msg.header.stamp.toSec();
    ides.drone_id = -1; // -1 is self drone;
    ides.pose_cam = fromROSPose(msg.pose_cam);
    ides.pose_drone = fromROSPose(msg.pose_drone);
    ides.landmark_num = msg.feature_points_2d_uv.size();
    ides.landmark_descriptor_length = ides.landmark_num*ORB_FEATURE_SIZE;

    ROSPoints2LCM(msg.feature_points_2d_norm, ides.landmarks_2d_norm);
    ROSPoints2LCM(msg.feature_points_3d, ides.landmarks_3d);

    auto desc = landmark_desc_compute(img, msg.feature_points_2d_uv);
    std::cout << "ORB DESC " << desc.size() << std::endl;
    ides.landmarks_descriptors = std::vector<unsigned char>(desc.data, desc.data + ides.landmark_num*ORB_FEATURE_SIZE);

    return std::pair<ImageDescriptor_t, cv::Mat> (ides, img);
}


cv::Mat LoopCam::landmark_desc_compute(const cv::Mat & _img, const std::vector<geometry_msgs::Point32> & points_uv) {
    cv::Mat ret;
    ROS_INFO("Compute %d landmark desc...", points_uv.size());
    auto _des = cv::ORB::create(LOOP_FEATURE_NUM);
    std::vector<cv::KeyPoint> kps;
    for (auto pt : points_uv) {
        cv::KeyPoint kp;
        kp.pt.x = pt.x;
        kp.pt.y = pt.y;
        kps.push_back(kp);
        // printf("Landmark on %f %f   ", pt.x, pt.y);
    }

    _des->compute(_img, kps, ret);
    // std::cout << "DESC 0" << desc.size() << std::endl;
    return ret;
}


ImageDescriptor_t LoopCam::feature_detect(const cv::Mat & _img) {
    ImageDescriptor_t img_des;
    std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

    // cv::FAST(_img, keypoints, FAST_THRES, true);

#ifdef USE_CUDA
    cv::cuda::GpuMat cuda_img(_img);
    cv::cuda::GpuMat mask;
    auto _des = cv::cuda::ORB::create(LOOP_FEATURE_NUM);
    _des->detectAndCompute(cuda_img, mask, keypoints, descriptors);

#else
    //TODO:Mask drones
    cv::Mat mask;
    auto _des = cv::ORB::create(LOOP_FEATURE_NUM);
    _des->detectAndCompute(_img, mask, keypoints, descriptors);
#endif
    // std::cout << "Features " << keypoints.size();
	
    for (int i = 0; i < (int)keypoints.size(); i++) {
		Eigen::Vector3d tmp_p;
		cam->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);
        Point2d_t point_2d_norm;
        point_2d_norm.x = tmp_p.x()/tmp_p.z();
        point_2d_norm.y = tmp_p.y()/tmp_p.z();
        img_des.all_features_2d_norm[i] = point_2d_norm;
	}

    // std::cout << "Features " << keypoints.size() <<  "; DIMS " << descriptors.dims <<  "; COLS " << descriptors.cols << " ROWS" << descriptors.rows << std::endl;
    // std::cout << "Type" << descriptors.type() << std::endl;
    // std::cout << descriptors.row(500);
    //TODO: Optimize copy here
    memcpy(img_des.feature_descriptor, descriptors.data, ORB_FEATURE_SIZE*LOOP_FEATURE_NUM);
    // img_des.feature_descriptor = std::vector<uint8_t>(descriptors.data, descriptors.data + ORB_FEATURE_SIZE*LOOP_FEATURE_NUM);
    return img_des;
}

cv::Mat LoopCam::pop_image_ts(ros::Time ts) {
    ROS_INFO("Pop image... queue len %d", image_queue.size());
    double ts_sec = ts.toSec();
    if (image_queue.size() == 0) {
        ROS_INFO("Pop image with NOTHING");
        cv::Mat ret;
        return ret;
    }
    // We assume here the timestamp of KF message is latter than image
    // TS <= all in image queue
    while (image_queue.size() > 0 && image_ts_queue[0] <  ts_sec - 0.001) {
        //Pop head
        image_queue.erase(image_queue.begin());
        image_ts_queue.erase(image_ts_queue.begin());
    }

    if (fabs(image_ts_queue[0] -  ts_sec ) < 0.001) {
        ROS_INFO("Pop image with dt %3.2fms", fabs(image_ts_queue[0] -  ts_sec )*1000);
        image_queue.erase(image_queue.begin());
        image_ts_queue.erase(image_ts_queue.begin());
        return image_queue[0];
    }

    ROS_ERROR("Can't found image!!!");
    cv::Mat ret;
    return ret;
}
