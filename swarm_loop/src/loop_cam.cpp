#include <loop_cam.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

LoopCam::LoopCam(const std::string & camera_config_path, int _loop_duration):
    loop_duration(_loop_duration)
{
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
    feature_detect(img);
}

ImageDescriptor LoopCam::feature_detect(const cv::Mat & _img) {
    //TODO:
    ImageDescriptor img_des;
    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(_img, keypoints, FAST_THRES, true);


    return img_des;
}

Eigen::Vector2d LoopCam::project_point(const Eigen::Vector2d & xy) {
    //TODO:
}

cv::Mat & LoopCam::pop_image_ts(ros::Time ts) {

}
