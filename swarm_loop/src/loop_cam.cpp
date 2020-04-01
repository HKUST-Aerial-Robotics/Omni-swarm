#include <loop_cam.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <chrono>


using namespace std::chrono; 

LoopCam::LoopCam(const std::string & camera_config_path, const std::string & BRIEF_PATTERN_FILE, int _self_id, ros::NodeHandle & nh): self_id(_self_id) {
    camodocal::CameraFactory cam_factory;
    cam = cam_factory.generateCameraFromYamlFile(camera_config_path);
    deepnet_client = nh.serviceClient<HFNetSrv>("/swarm_loop/hfnet");
    printf("Waiting for deepnet......\n");
    deepnet_client.waitForExistence();
    printf("Deepnet ready\n");
}

cv::Point2d LoopCam::project_to_norm2d(cv::Point2f p) {
    Eigen::Vector3d tmp_p;
    cam->liftProjective(Eigen::Vector2d(p.x, p.y), tmp_p);
    cv::Point2f ret;
    ret.x = tmp_p.x()/tmp_p.z();
    ret.y = tmp_p.y()/tmp_p.z();

    return ret;
}

void LoopCam::encode_image(cv::Mat & _img, ImageDescriptor_t & _img_desc) {
    auto start = high_resolution_clock::now();
    
    std::vector<int> params;
    params.push_back( cv::IMWRITE_JPEG_QUALITY );
    params.push_back( JPG_QUALITY );

    cv::imencode(".jpg", _img, _img_desc.image, params);
    // std::cout << "IMENCODE Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 << "ms" << std::endl;
    // std::cout << "JPG SIZE" << _img_desc.image.size() << std::endl;

    _img_desc.image_height = _img.size().height;
    _img_desc.image_width = _img.size().width;
    _img_desc.image_size = _img_desc.image.size();
}


/*
ImageDescriptor_t  LoopCam::on_keyframe_message(const vins::VIOKeyframe& msg, cv::Mat & img){
    ROS_INFO("Received new keyframe. with %ld landmarks...", msg.feature_points_2d_uv.size());
    
    img = pop_image_ts(msg.header.stamp);
    ImageDescriptor_t ides;

    ides.landmark_num = 0;
    ides.image_desc_size = 0;
    ides.feature_descriptor_size = 0;
    ides.image_size = 0;
    
    if (img.empty()) {
        ROS_WARN("No Image; Exiting;");
        return ides;
    }

    auto start = high_resolution_clock::now();

#ifdef DEBUG_SHOW_IMAGE
    ROSPoints2LCM(msg.feature_points_2d_uv, ides.landmarks_2d);
    auto nowPts = toCV(ides.landmarks_2d);
    cv::Mat img_color;
    cv::cvtColor(img, img_color, cv::COLOR_GRAY2BGR);

    for (auto pt: nowPts) {
            // std::cout << pt << std::endl;
        cv::circle(img_color, pt, 2, cv::Scalar(0,0, 255), -1);
    }

    cv::resize(img_color, img_color, cv::Size(), 2, 2);
    cv::imshow("img", img_color);
    cv::waitKey(30);
#endif

#ifdef USE_DEEPNET
    // ides = extractor_img_desc_deepnet(msg.header.stamp);
    if (ides.image_desc_size == 0) {
        ROS_WARN("Failed on deepnet;");
        cv::Mat _img;
        return ides;
    }
#else
    ides = extractor_img_desc(img);
    if (ides.feature_descriptor_size == 0) {
        return ides;
    }
#endif
    // std::cout << "FeatureDetect Cost " << duration_cast<milliseconds>(high_resolution_clock::now() - start).count() << "ms" << std::endl;

    start = high_resolution_clock::now();
    cv::resize(img, img, img.size()/LOOP_IMAGE_DOWNSAMPLE);
    encode_image(img, ides);
    // std::cout << "Downsample and encode Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 << "ms" << std::endl;

    ides.timestamp = toLCMTime(msg.header.stamp);
    ides.drone_id = self_id; // -1 is self drone;
    ides.camera_extrinsic = fromROSPose(msg.camera_extrisinc);
    ides.pose_drone = fromROSPose(msg.pose_drone);
    ides.landmark_num = msg.feature_points_2d_uv.size();
    // ides.landmark_descriptor_length = ides.landmark_num*ORB_FEATURE_SIZE;
    ROSPoints2LCM(msg.feature_points_2d_norm, ides.landmarks_2d_norm);
    ROSPoints2LCM(msg.feature_points_2d_uv, ides.landmarks_2d);
    ROSPoints2LCM(msg.feature_points_3d, ides.landmarks_3d);
    ides.landmarks_flag = msg.feature_points_flag;
   
    return ides;
}
*/

ImageDescriptor_t LoopCam::on_flattened_images(const vins::FlattenImages& msg) {
    auto ides = extractor_img_desc_deepnet(msg.header.stamp, msg.up_cams[0]);
    if (ides.image_desc_size == 0) {
        ROS_WARN("Failed on deepnet;");
        cv::Mat _img;
        return ides;
    }

    auto start = high_resolution_clock::now();
    // std::cout << "Downsample and encode Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 << "ms" << std::endl;

    ides.timestamp = toLCMTime(msg.header.stamp);
    ides.drone_id = self_id; // -1 is self drone;
    ides.camera_extrinsic = fromROSPose(msg.pose_up_cams[0]);
    ides.pose_drone = fromROSPose(msg.pose_drone);
    // ides.landmark_num = msg.feature_points_2d_uv.size();
    // ides.landmark_descriptor_length = ides.landmark_num*ORB_FEATURE_SIZE;
   
    //Need to triangulate 2 3d position here
    return ides;
}


cv::Mat LoopCam::landmark_desc_compute(const cv::Mat & _img, const std::vector<geometry_msgs::Point32> & points_uv) {
    cv::Mat ret;
    ROS_INFO("Compute %ld landmark desc...", points_uv.size());
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


ImageDescriptor_t LoopCam::extractor_img_desc_deepnet(ros::Time stamp, const sensor_msgs::Image& msg) {
    auto start = high_resolution_clock::now();

    ImageDescriptor_t img_des;
    img_des.image_desc_size = 0;
    img_des.feature_descriptor_size  = 0;
    img_des.image_size  = 0;
    img_des.landmark_num = 0;

    HFNetSrv hfnet_srv;
    hfnet_srv.request.image = msg;

    if( deepnet_client.call( hfnet_srv ) ) {
        auto & desc = hfnet_srv.response.global_desc;
        auto & local_kpts = hfnet_srv.response.keypoints;
        auto & local_descriptors = hfnet_srv.response.local_descriptors;
        if (desc.size() > 0) {
            // ROS_INFO("Received response from server desc.size %ld", desc.size());
            img_des.image_desc_size = desc.size();
            img_des.image_desc = desc;
            img_des.feature_descriptor = local_descriptors;
        } else {
            ROS_WARN("Failed on deepnet; Please check deepnet queue");
        }

        return img_des;
    } else {
        ROS_INFO("FAILED on deepnet!!! Service error");
        return img_des;
    }
    return img_des;
}


std::vector<cv::Point2f> LoopCam::project_to_image(std::vector<cv::Point2f> points_norm2d) {
    std::vector<cv::Point2f> ret;
    for (auto pt : points_norm2d) {
        Eigen::Vector3d p3d(pt.x, pt.y, 1);
        Eigen::Vector2d p2d;
        cam->spaceToPlane(p3d, p2d);
        cv::Point2f cvpt2d;
        cvpt2d.x = p2d.x();
        cvpt2d.y = p2d.y();

        ret.push_back(cvpt2d);
    }

    return ret;
}
