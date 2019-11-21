#include <loop_cam.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <swarm_msgs/swarm_lcm_converter.hpp>

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

ImageDescriptor_t LoopCam::on_keyframe_message(const vins::VIOKeyframe& msg) {
    cv::Mat & img = pop_image_ts(msg.header.stamp);
    ImageDescriptor_t ides;
    if (img.dims == 0) {
        return ides;
    }

    ides = feature_detect(img);

    ides.timestamp = msg.header.stamp.toSec();
    ides.drone_id = -1; // -1 is self drone;
    ides.pose_cam = fromROSPose(msg.pose_cam);
    ides.pose_drone = fromROSPose(msg.pose_drone);

    ROSPoints2LCM(msg.feature_point_2d_norm, ides.keyfeature_point_2d_norm);
    ROSPoints2LCM(msg.feature_point_3d, ides.keyfeature_point_3d);

    return ides;
}

ImageDescriptor_t LoopCam::feature_detect(const cv::Mat & _img) {
    auto _des = cv::ORB::create(LOOP_FEATURE_NUM);
    ImageDescriptor_t img_des;
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
        Point2d_t point_2d_norm;
        point_2d_norm.x = tmp_p.x()/tmp_p.z();
        point_2d_norm.y = tmp_p.y()/tmp_p.z();
        img_des.all_features_2d[i] = point_2d_norm;
	}

    // std::cout << "Features " << keypoints.size() <<  "; DIMS " << descriptors.dims <<  "; COLS " << descriptors.cols << " ROWS" << descriptors.rows << std::endl;
    // std::cout << "Type" << descriptors.type() << std::endl;
    // std::cout << descriptors.row(500);
    //TODO: Optimize copy here
    memcpy(img_des.feature_descriptor, descriptors.data, ORB_FEATURE_SIZE*LOOP_FEATURE_NUM);
    // img_des.feature_descriptor = std::vector<uint8_t>(descriptors.data, descriptors.data + ORB_FEATURE_SIZE*LOOP_FEATURE_NUM);
    return img_des;
}

cv::Mat & LoopCam::pop_image_ts(ros::Time ts) {
    double ts_sec = ts.toSec();
    // We assume here the timestamp of KF message is latter than image
    // TS <= all in image queue
    while (image_queue.size() > 0 && image_ts_queue[0] <  ts_sec - 0.001) {
        //Pop head
        image_queue.erase(image_queue.begin());
        image_ts_queue.erase(image_ts_queue.begin());
    }

    if (fabs(image_ts_queue[0] -  ts_sec ) < 0.001) {
        cv::Mat ret = image_queue[0];
        //Check here!
        ROS_INFO("Pop image with dt %3.2fms", fabs(image_ts_queue[0] -  ts_sec )*1000);
        image_queue.erase(image_queue.begin());
        image_ts_queue.erase(image_ts_queue.begin());
        return ret;
    }

    ROS_ERROR("Can't found image!!!");
    cv::Mat ret;
    return ret;
}
