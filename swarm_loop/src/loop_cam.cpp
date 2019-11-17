#include <loop_cam.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

LoopCam::LoopCam(const std::string & camera_config_path, const std::string & BRIEF_PATTERN_FILE) {
    camodocal::CameraFactory cam_factory;
    cam = cam_factory.generateCameraFromYamlFile(camera_config_path);
}

void LoopCam::on_camera_message(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    image_queue.push_back(ptr->image);
    cam_count ++;
}

void LoopCam::on_keyframe_message(const vins::VIOKeyframe& msg) {
    cv::Mat & img = pop_image_ts(msg.header.stamp);
    auto ides = feature_detect(img);

    ides.header.stamp = msg.header.stamp;
    ides.drone_id = -1; // -1 is self drone;
    ides.pose_cam = msg.pose_cam;
    ides.pose_drone = msg.pose_drone;
    ides.keyfeature_point_2d_norm = msg.feature_point_2d_norm;
    ides.keyfeature_point_3d = msg.feature_point_3d;
}

ImageDescriptor LoopCam::feature_detect(const cv::Mat & _img) {
    auto _des = cv::ORB::create(LOOP_FEATURE_NUM);
    ImageDescriptor img_des;
    std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

    // cv::FAST(_img, keypoints, FAST_THRES, true);

    cv::Mat mask;
    //TODO:Mask drones
    _des->detectAndCompute(_img, mask, keypoints, descriptors);
    
    // std::cout << "Features " << keypoints.size();
	
    for (int i = 0; i < (int)keypoints.size(); i++) {
		Eigen::Vector3d tmp_p;
		cam->liftProjective(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);
		cv::KeyPoint tmp_norm;
        geometry_msgs::Point32 point_2d_norm;
        point_2d_norm.x = tmp_p.x()/tmp_p.z();
        point_2d_norm.y = tmp_p.y()/tmp_p.z();
        point_2d_norm.z = 1.0;
        img_des.all_features_2d.push_back(point_2d_norm);
	}

	// _des->compute( _img, keypoints, descriptors);
    
    // std::cout << "Features " << keypoints.size() <<  "; DIMS " << descriptors.dims <<  "; COLS " << descriptors.cols << " ROWS" << descriptors.rows << std::endl;
    // std::cout << "Type" << descriptors.type() << std::endl;
    // std::cout << descriptors.row(500);
    img_des.feature_descriptor = std::vector<uint8_t>(descriptors.data, descriptors.data + ORB_FEATURE_SIZE*LOOP_FEATURE_NUM);
    return std::move(img_des);
}

cv::Mat & LoopCam::pop_image_ts(ros::Time ts) {
    //TODO:
}
