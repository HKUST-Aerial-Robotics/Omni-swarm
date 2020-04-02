#include <loop_cam.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <chrono>

using namespace std::chrono;

LoopCam::LoopCam(const std::string &camera_config_path, const std::string &BRIEF_PATTERN_FILE, int _self_id, ros::NodeHandle &nh) : self_id(_self_id)
{
    camodocal::CameraFactory cam_factory;
    cam = cam_factory.generateCameraFromYamlFile(camera_config_path);
    deepnet_client = nh.serviceClient<HFNetSrv>("/swarm_loop/hfnet");
    printf("Waiting for deepnet......\n");
    deepnet_client.waitForExistence();
    printf("Deepnet ready\n");
}

cv::Point2d LoopCam::project_to_norm2d(cv::Point2f p)
{
    Eigen::Vector3d tmp_p;
    cam->liftProjective(Eigen::Vector2d(p.x, p.y), tmp_p);
    cv::Point2f ret;
    ret.x = tmp_p.x() / tmp_p.z();
    ret.y = tmp_p.y() / tmp_p.z();

    return ret;
}

void LoopCam::encode_image(cv::Mat &_img, ImageDescriptor_t &_img_desc)
{
    auto start = high_resolution_clock::now();

    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(JPG_QUALITY);

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

void triangulatePoint(Eigen::Quaterniond q0, Eigen::Vector3d t0, Eigen::Quaterniond q1, Eigen::Vector3d t1,
                      Eigen::Vector2d point0, Eigen::Vector2d point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix3d R0 = q0.toRotationMatrix();
    Eigen::Matrix3d R1 = q1.toRotationMatrix();

    // std::cout << "RO" << R0 << "T0" << t0.transpose() << std::endl;
    // std::cout << "R1" << R1 << "T1" << t1.transpose() << std::endl;

    Eigen::Matrix<double, 3, 4> Pose0;
    Pose0.leftCols<3>() = R0.transpose();
    Pose0.rightCols<1>() = -R0.transpose() * t0;

    Eigen::Matrix<double, 3, 4> Pose1;
    Pose1.leftCols<3>() = R1.transpose();
    Pose1.rightCols<1>() = -R1.transpose() * t1;

    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
        design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void track_pts(cv::Mat &img_up, cv::Mat &img_down, std::vector<cv::Point2f> &pts_up, std::vector<cv::Point2f> &pts_down)
{
    std::vector<float> err;
    std::vector<uchar> status;
    std::cout << "DOWN " << img_down.size() << " Up" << img_up.size() << "Pts " << pts_up.size() << std::endl;

    cv::calcOpticalFlowPyrLK(img_up, img_down, pts_up, pts_down, status, err, cv::Size(21, 21), 3);
    // reduceVector(pts_down, status);
    // reduceVector(pts_up, status);

    std::vector<cv::Point2f> reverse_pts;
    std::vector<uchar> reverse_status;
    cv::calcOpticalFlowPyrLK(img_down, img_up, pts_down, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);

    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i] && reverse_status[i] && cv::norm(pts_up[i] - reverse_pts[i]) <= 0.5)
        {
            status[i] = 1;
        }
        else
        {
            status[i] = 0;
        }
    }

    reduceVector(pts_down, status);
    reduceVector(pts_up, status);
}

ImageDescriptor_t LoopCam::on_flattened_images(const vins::FlattenImages &msg, cv::Mat & img)
{
    auto ides = extractor_img_desc_deepnet(msg.header.stamp, msg.up_cams[0]);
    if (ides.image_desc_size == 0)
    {
        ROS_WARN("Failed on deepnet;");
        cv::Mat _img;
        return ides;
    }
    else
    {
        ROS_INFO("Deepnet gives %ld kpts", ides.landmarks_2d.size());
    }

    auto start = high_resolution_clock::now();
    // std::cout << "Downsample and encode Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 << "ms" << std::endl;

    ides.timestamp = toLCMTime(msg.header.stamp);
    ides.drone_id = self_id; // -1 is self drone;
    ides.camera_extrinsic = fromROSPose(msg.extrinsic_up_cams[0]);
    ides.pose_drone = fromROSPose(msg.pose_drone);

    auto cv_ptr = cv_bridge::toCvCopy(msg.up_cams[0], sensor_msgs::image_encodings::BGR8);
    cv::Mat img_up = cv_ptr->image;
    img_up.copyTo(img);

    auto cv_ptr2 = cv_bridge::toCvCopy(msg.down_cams[0], sensor_msgs::image_encodings::BGR8);
    cv::Mat img_down = cv_ptr2->image;

    std::vector<cv::Point2f> pts_up, pts_down;
    pts_up = toCV(ides.landmarks_2d);

    ides.landmarks_2d.clear();
    ides.landmarks_2d_norm.clear();
    ides.landmarks_3d.clear();

    track_pts(cv_ptr->image, cv_ptr2->image, pts_up, pts_down);
    ROS_INFO("tracked points %ld", pts_down.size());
    std::vector<Eigen::Vector3d> pts_3d;

    Swarm::Pose pose_drone(msg.pose_drone);
    Swarm::Pose pose_up = pose_drone * Swarm::Pose(msg.extrinsic_up_cams[0]);
    Swarm::Pose pose_down = pose_drone * Swarm::Pose(msg.extrinsic_down_cams[0]);

    for (unsigned int i = 0; i < pts_up.size(); i++)
    {
        auto pt_up = pts_up[i];
        auto pt_down = pts_down[i];

        Eigen::Vector3d pt_up3d, pt_down3d;
        cam->liftProjective(Eigen::Vector2d(pt_up.x, pt_up.y), pt_up3d);
        cam->liftProjective(Eigen::Vector2d(pt_down.x, pt_down.y), pt_down3d);

        Eigen::Vector2d pt_up_norm(pt_up3d.x()/pt_up3d.z(), pt_up3d.y()/pt_up3d.z());
        Eigen::Vector2d pt_down_norm(pt_down3d.x()/pt_down3d.z(), pt_down3d.y()/pt_down3d.z());

        Eigen::Vector3d point_3d;
        triangulatePoint(pose_up.att(), pose_up.pos(), pose_down.att(), pose_down.pos(),
                        pt_up_norm, pt_down_norm, point_3d);

        pts_3d.push_back(point_3d);


        Point2d_t pt2d;
        pt2d.x = pt_up.x;
        pt2d.y = pt_up.y;

        Point2d_t pt2d_norm;
        pt2d_norm.x = pt_up_norm.x();
        pt2d_norm.y = pt_up_norm.y();

        Point3d_t pt3d;
        pt3d.x = point_3d.x();
        pt3d.y = point_3d.y();
        pt3d.z = point_3d.z();

        ides.landmarks_2d.push_back(pt2d);
        ides.landmarks_2d_norm.push_back(pt2d_norm);
        ides.landmarks_3d.push_back(pt3d);

        // std::cout << "PT UP" << pt_up << "PT DOWN" << pt_down << std::endl;

        // std::cout << "PT UP NORM" << pt_up_norm.transpose() << "PT DOWN NORM" << pt_down_norm.transpose() << std::endl;

        // std::cout << "P3d:" << point_3d.transpose() << std::endl;
    }

    ides.landmark_num = ides.landmarks_2d.size();

    

    cv::Mat show;

    for (auto pt : pts_up)
    {
        cv::circle(img_up, pt, 1, cv::Scalar(255, 0, 0), -1);
    }

    for (auto pt : pts_down)
    {
        cv::circle(img_down, pt, 1, cv::Scalar(255, 0, 0), -1);
    }

    cv::hconcat(img_up, img_down, show);

    char text[100] = {0};

    // cv::resize(show, show, cv::Size(0, 0), 2, 2);

    for (unsigned int i = 0; i < pts_up.size(); i++) {
        cv::arrowedLine(show, pts_up[i], pts_down[i], cv::Scalar(255, 255, 0), 1);
    }

    // for (unsigned int i = 0; i < pts_up.size(); i++) {
    //     sprintf(text, "[%3.2f, %3.2f, %3.2f]", pts_3d[i].x(), pts_3d[i].y(), pts_3d[i].z());
	//     cv::putText(show, text, pts_up[i]*2 - cv::Point2f(0, 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
    // }

    ROS_INFO("Try show image");
    cv::imshow("DEBUG", show);
    cv::waitKey(10);

    return ides;
}

cv::Mat LoopCam::landmark_desc_compute(const cv::Mat &_img, const std::vector<geometry_msgs::Point32> &points_uv)
{
    cv::Mat ret;
    ROS_INFO("Compute %ld landmark desc...", points_uv.size());
    auto _des = cv::ORB::create(LOOP_FEATURE_NUM);
    std::vector<cv::KeyPoint> kps;
    for (auto pt : points_uv)
    {
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

ImageDescriptor_t LoopCam::extractor_img_desc_deepnet(ros::Time stamp, const sensor_msgs::Image &msg)
{
    auto start = high_resolution_clock::now();

    ImageDescriptor_t img_des;
    img_des.image_desc_size = 0;
    img_des.feature_descriptor_size = 0;
    img_des.image_size = 0;
    img_des.landmark_num = 0;

    HFNetSrv hfnet_srv;
    hfnet_srv.request.image = msg;

    if (deepnet_client.call(hfnet_srv))
    {
        auto &desc = hfnet_srv.response.global_desc;
        auto &local_kpts = hfnet_srv.response.keypoints;
        auto &local_descriptors = hfnet_srv.response.local_descriptors;
        if (desc.size() > 0)
        {
            // ROS_INFO("Received response from server desc.size %ld", desc.size());
            img_des.image_desc_size = desc.size();
            img_des.image_desc = desc;
            ROSPoints2LCM(local_kpts, img_des.landmarks_2d);
            img_des.landmark_num = local_kpts.size();
            img_des.feature_descriptor = local_descriptors;
            img_des.feature_descriptor_size = local_descriptors.size();
            img_des.image_size = 0;
        }
        else
        {
            ROS_WARN("Failed on deepnet; Please check deepnet queue");
        }

        return img_des;
    }
    else
    {
        ROS_INFO("FAILED on deepnet!!! Service error");
        return img_des;
    }
    return img_des;
}

std::vector<cv::Point2f> LoopCam::project_to_image(std::vector<cv::Point2f> points_norm2d)
{
    std::vector<cv::Point2f> ret;
    for (auto pt : points_norm2d)
    {
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
