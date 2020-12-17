#include <loop_cam.h>
#include <camodocal/camera_models/CameraFactory.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <chrono>
#include <opencv2/core/eigen.hpp>

using namespace std::chrono;

double TRIANGLE_THRES;

LoopCam::LoopCam(const std::string &camera_config_path, const std::string &superpoint_model, double thres, 
    const std::string & netvlad_model, int width, int height, int _self_id, bool _send_img, ros::NodeHandle &nh) : 
    self_id(_self_id),
#ifdef USE_TENSORRT
    superpoint_net(superpoint_model, width, height, thres), 
    netvlad_net(netvlad_model, width, height), 
#endif
    send_img(_send_img)
{
    camodocal::CameraFactory cam_factory;
    ROS_INFO("Read camera from %s", camera_config_path.c_str());
    cam = cam_factory.generateCameraFromYamlFile(camera_config_path);

#ifndef USE_TENSORRT
    hfnet_client = nh.serviceClient<HFNetSrv>("/swarm_loop/hfnet");
    superpoint_client = nh.serviceClient<HFNetSrv>("/swarm_loop/superpoint");
#endif
    camodocal::PinholeCamera* _cam = (camodocal::PinholeCamera*)cam.get();

    Eigen::Matrix3d _cameraMatrix;
    _cameraMatrix << _cam->getParameters().fx(), 0, _cam->getParameters().cx(),
                    0, _cam->getParameters().fy(), _cam->getParameters().cy(), 0, 0, 1;
    cv::eigen2cv(_cameraMatrix, cameraMatrix);

    printf("Waiting for deepnet......\n");
    hfnet_client.waitForExistence();
    superpoint_client.waitForExistence();
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

void LoopCam::encode_image(const cv::Mat &_img, ImageDescriptor_t &_img_desc)
{
    auto start = high_resolution_clock::now();

    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(JPG_QUALITY);

    cv::imencode(".jpg", _img, _img_desc.image, params);
    // std::cout << "IMENCODE Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 << "ms" << std::endl;
    std::cout << "JPG SIZE" << _img_desc.image.size() << std::endl;

    _img_desc.image_height = _img.size().height;
    _img_desc.image_width = _img.size().width;
    _img_desc.image_size = _img_desc.image.size();
}

double triangulatePoint(Eigen::Quaterniond q0, Eigen::Vector3d t0, Eigen::Quaterniond q1, Eigen::Vector3d t1,
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

    Eigen::MatrixXd pts(4, 1);
    pts << point_3d.x(), point_3d.y(), point_3d.z(), 1;
    Eigen::MatrixXd errs = design_matrix*pts;
    return errs.norm()/ errs.rows(); 
}

template <typename T>
void reduceVector(std::vector<T> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

cv::Mat drawMatches(std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, std::vector<cv::DMatch> _matches, const cv::Mat & up, const cv::Mat & down) {
    std::vector<cv::KeyPoint> kps1;
    std::vector<cv::KeyPoint> kps2;

    for (auto pt : pts1) {
        cv::KeyPoint kp;
        kp.pt = pt;
        kps1.push_back(kp);
    }

    for (auto pt : pts2) {
        cv::KeyPoint kp;
        kp.pt = pt;
        kps2.push_back(kp);
    }

    cv::Mat _show;

    cv::drawMatches(up, kps1, down, kps2, _matches, _show);

    return _show;
}

std::vector<int> LoopCam::match_HFNet_local_features(std::vector<cv::Point2f> & pts_up, std::vector<cv::Point2f> & pts_down, std::vector<float> _desc_up, std::vector<float> _desc_down,
        const cv::Mat & up, const cv::Mat & down) {
    printf("match_HFNet_local_features %ld %ld: ", pts_up.size(), pts_down.size());
    cv::Mat desc_up( _desc_up.size()/LOCAL_DESC_LEN, LOCAL_DESC_LEN, CV_32F, _desc_up.data());
    cv::Mat desc_down( _desc_down.size()/LOCAL_DESC_LEN, LOCAL_DESC_LEN, CV_32F, _desc_down.data());

    cv::BFMatcher bfmatcher(cv::NORM_L2, true);

    std::vector<cv::DMatch> _matches;
    bfmatcher.match(desc_up, desc_down, _matches);

    std::vector<cv::Point2f> _pts_up, _pts_down;
    std::vector<int> ids;
    for (auto match : _matches) {
        if (match.distance < ACCEPT_SP_MATCH_DISTANCE || true) 
        {
            int now_id = match.queryIdx;
            int old_id = match.trainIdx;
            _pts_up.push_back(pts_up[now_id]);
            _pts_down.push_back(pts_down[old_id]);
            ids.push_back(now_id);
        } else {
            std::cout << "Giveup match dis" << match.distance << std::endl;
        }
    }

    printf("%ld matches...", _matches.size());

    std::vector<uint8_t> status;
    // cv::findEssentialMat(_pts_up, _pts_down, cameraMatrix, cv::RANSAC, 0.999, 1.0, status);
    // cv::findFundamentalMat(_pts_up, _pts_down, cv::FM_RANSAC, 1.0, 0.99, status);

    // reduceVector(_pts_up, status);
    // reduceVector(_pts_down, status);
    // reduceVector(ids, status);
    // reduceVector(_matches, status);


    // if (show) {
    //     cv::Mat img = drawMatches(pts_up, pts_down, _matches, up, down);
    //     cv::imshow("Stereo Matches", img);
    //     cv::waitKey(30);
    // }

    pts_up = std::vector<cv::Point2f>(_pts_up);
    pts_down = std::vector<cv::Point2f>(_pts_down);
    return ids;
}



FisheyeFrameDescriptor_t LoopCam::on_flattened_images(const vins::FlattenImages & msg, std::vector<cv::Mat> imgs) {
    FisheyeFrameDescriptor_t frame_desc;
    
    if (msg.up_cams[3].width > 0) {
        imgs.resize(4);
    } else {
        imgs.resize(3);
    }

    frame_desc.images.push_back(generate_image_descriptor(msg, imgs[0], 1));
    frame_desc.images.push_back(generate_image_descriptor(msg, imgs[1], 2));
    frame_desc.images.push_back(generate_image_descriptor(msg, imgs[2], 3));
    
    if (msg.up_cams[3].width > 0) {
        frame_desc.images.push_back(generate_image_descriptor(msg, imgs[3], 4));
    } else {
        frame_desc.images.push_back(generate_null_img_desc());
    }
    
    frame_desc.image_num = 4;
    frame_desc.timestamp = frame_desc.images[0].timestamp;
    frame_desc.images[1].timestamp = frame_desc.timestamp;
    frame_desc.images[2].timestamp = frame_desc.timestamp;
    frame_desc.images[3].timestamp = frame_desc.timestamp;

    for (size_t i = 0; i < 4; i++) {
        frame_desc.images[i].direction = i;
    }
    
    frame_desc.msg_id = frame_desc.timestamp.nsec%100000 * 10000 + rand()%10000 + self_id * 100;
    frame_desc.pose_drone = fromROSPose(msg.pose_drone);
    frame_desc.landmark_num = 0;
    for (auto & frame : frame_desc.images) {
        frame_desc.landmark_num += frame.landmark_num;
    }
    frame_desc.drone_id = self_id;
    return frame_desc;
}

ImageDescriptor_t LoopCam::generate_image_descriptor(const vins::FlattenImages & msg, cv::Mat & img, const int & vcam_id)
{
    if (vcam_id > msg.up_cams.size()) {
        ROS_WARN("Flatten images too few");
        ImageDescriptor_t ides;
        ides.landmark_num = 0;
        return ides;
    }
    
    auto ides = extractor_img_desc_deepnet(msg.header.stamp, msg.up_cams[vcam_id]);
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
    ides.camera_extrinsic = fromROSPose(msg.extrinsic_up_cams[vcam_id]);
    ides.pose_drone = fromROSPose(msg.pose_drone);
    ides.image_size = 0;
    auto cv_ptr = cv_bridge::toCvShare(msg.up_cams[vcam_id], boost::make_shared<vins::FlattenImages>(msg));
    auto cv_ptr2 = cv_bridge::toCvShare(msg.down_cams[vcam_id], boost::make_shared<vins::FlattenImages>(msg));

    std::vector<cv::Point2f> pts_up, pts_down;
    pts_up = toCV(ides.landmarks_2d);

    std::vector<int> ids;

    if (pts_down.size() < ACCEPT_MIN_3D_PTS) {
        pts_up = toCV(ides.landmarks_2d);
        auto ides_down = extractor_img_desc_deepnet(msg.header.stamp, msg.down_cams[vcam_id], true);
        pts_down = toCV(ides_down.landmarks_2d);

        cv::Mat _img = cv_ptr->image;
        cv::Mat _img2 = cv_ptr2->image;

        if (show) {
            cv::Mat img_show;
            cv::cvtColor(_img, img_show, cv::COLOR_GRAY2BGR);
            for (auto pt : pts_up) {
                cv::circle(img_show, pt, 1, cv::Scalar(255, 0, 0), -1);
            }
        }

        ids = match_HFNet_local_features(pts_up, pts_down, ides.feature_descriptor, ides_down.feature_descriptor, _img, _img2);
    }
    
    // ides.landmarks_2d.clear();
    // ides.landmarks_2d_norm.clear();
    // ides.landmarks_3d.clear();
    
    std::vector<Eigen::Vector3d> pts_3d;

    Swarm::Pose pose_drone(msg.pose_drone);
    Swarm::Pose pose_up = pose_drone * Swarm::Pose(msg.extrinsic_up_cams[vcam_id]);
    Swarm::Pose pose_down = pose_drone * Swarm::Pose(msg.extrinsic_down_cams[vcam_id]);

    std::vector<float> desc_new;

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
        double err = triangulatePoint(pose_up.att(), pose_up.pos(), pose_down.att(), pose_down.pos(),
                        pt_up_norm, pt_down_norm, point_3d);

        // std::cout << "Pt: " << i << "Pos " << point_3d.transpose() << " err " << err << std::endl;

        if (err > TRIANGLE_THRES) {
            continue;
        }

        pts_3d.push_back(point_3d);


        Point2d_t pt2d;
        pt2d.x = pt_up.x;
        pt2d.y = pt_up.y;

        Point3d_t pt3d;
        pt3d.x = point_3d.x();
        pt3d.y = point_3d.y();
        pt3d.z = point_3d.z();

        int idx = ids[i];
        // ides.landmarks_2d.push_back(pt2d);
        // ides.landmarks_2d_norm.push_back(pt2d_norm);
        ides.landmarks_3d[idx] = pt3d;
        ides.landmarks_flag[idx] = 1;

        // std::cout << "Insert" << LOCAL_DESC_LEN * ids[i] << "to" << LOCAL_DESC_LEN * (ids[i] + 1)  << std::endl;

        // desc_new.insert(desc_new.end(), ides.feature_descriptor.begin() + LOCAL_DESC_LEN * ids[i], ides.feature_descriptor.begin() + LOCAL_DESC_LEN * (ids[i] + 1) );

        // std::cout << "PT UP" << pt_up << "PT DOWN" << pt_down << std::endl;

        // std::cout << "PT UP NORM" << pt_up_norm.transpose() << "PT DOWN NORM" << pt_down_norm.transpose() << std::endl;

    }

    printf("3D features: %d\n", pts_3d.size());

    // ides.feature_descriptor.clear();
    // ides.feature_descriptor = std::vector<float>(desc_new);
    // ides.feature_descriptor_size = ides.feature_descriptor.size();
    // ides.landmark_num = ides.landmarks_2d.size();

    if (send_img) {
        encode_image(cv_ptr->image, ides);
    }

    if (show) {
        cv::Mat img_up = cv_ptr->image;
        cv::Mat img_down = cv_ptr2->image;

        img_up.copyTo(img);
        encode_image(img_up, ides);

        cv::Mat show;
        cv::cvtColor(img_up, img_up, cv::COLOR_GRAY2BGR);
        cv::cvtColor(img_down, img_down, cv::COLOR_GRAY2BGR);

        for (auto pt : pts_down)
        {
            cv::circle(img_down, pt, 1, cv::Scalar(255, 0, 0), -1);
        }

        for (auto _pt : ides.landmarks_2d) {
            cv::Point2f pt(_pt.x, _pt.y);
            cv::circle(img_up, pt, 3, cv::Scalar(0, 0, 255), 1);
        }

        cv::hconcat(img_up, img_down, show);

        char text[100] = {0};

        // cv::resize(show, show, cv::Size(0, 0), 2, 2);

        for (unsigned int i = 0; i < pts_up.size(); i++) {
            cv::arrowedLine(show, pts_up[i], pts_down[i], cv::Scalar(255, 255, 0), 1);
        }

        // cv::resize(show, show, cv::Size(), 2, 2);

        for (unsigned int i = 0; i < pts_up.size(); i++)
        {
            char title[100] = {0};
            auto pt = pts_up[i];
            sprintf(title, "%d", i);
            cv::putText(show, title, pt + cv::Point2f(0, 10), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        sprintf(text, "FEATURE CAM %d", vcam_id);
        
        cv::imshow(text, show);
        cv::waitKey(10);
    }
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

ImageDescriptor_t LoopCam::extractor_img_desc_deepnet(ros::Time stamp, const sensor_msgs::Image &msg, bool superpoint_mode)
{
    auto start = high_resolution_clock::now();

    ImageDescriptor_t img_des;
    img_des.image_desc_size = 0;
    img_des.feature_descriptor_size = 0;
    img_des.image_size = 0;
    img_des.landmark_num = 0;



    auto cv_ptr = cv_bridge::toCvCopy(msg);
    cv::Mat roi = cv_ptr->image(cv::Rect(0, cv_ptr->image.rows*3/4, cv_ptr->image.cols, cv_ptr->image.rows/4));
    roi.setTo(cv::Scalar(0, 0, 0));
    // std::cout << "Image size" << cv_ptr->image.size() << std::endl;
#ifdef USE_TENSORRT
    std::vector<cv::Point2f> features;
    superpoint_net.inference(cv_ptr->image, features, img_des.feature_descriptor);
    img_des.image_desc_size = 0;
    img_des.image_desc.clear();
    CVPoints2LCM(features, img_des.landmarks_2d);
    img_des.landmark_num = features.size();
    img_des.feature_descriptor_size =  img_des.feature_descriptor.size();
    img_des.landmarks_flag.clear();
    img_des.landmarks_3d.clear();
    img_des.landmarks_2d_norm.clear();
    img_des.image_size = 0;

    if (!superpoint_mode) {
        img_des.image_desc = netvlad_net.inference(cv_ptr->image);
        img_des.image_desc_size = img_des.image_desc.size();
    }
    // if (hfnet_client.call(hfnet_srv))
    // {
    //     auto &desc = hfnet_srv.response.global_desc;
    //     if (desc.size() > 0)
    //     {
    //         // ROS_INFO("Received response from server desc.size %ld", desc.size());
    //         img_des.image_desc_size = desc.size();
    //         img_des.image_desc = desc;
    //         return img_des;
    //     }
    // }
    for (unsigned int i = 0; i < img_des.landmarks_2d.size(); i++)
    {
        auto pt_up = img_des.landmarks_2d[i];
        Eigen::Vector3d pt_up3d;
        Point2d_t pt2d_norm;
        cam->liftProjective(Eigen::Vector2d(pt_up.x, pt_up.y), pt_up3d);
        Eigen::Vector2d pt_up_norm(pt_up3d.x()/pt_up3d.z(), pt_up3d.y()/pt_up3d.z());

        pt2d_norm.x = pt_up_norm.x();
        pt2d_norm.y = pt_up_norm.y();

        img_des.landmarks_2d_norm.push_back(pt2d_norm);
        Point3d_t pt3d;
        pt3d.x = 0;
        pt3d.y = 0;
        pt3d.z = 0;
        img_des.landmarks_3d.push_back(pt3d);
        img_des.landmarks_flag.push_back(0);
    } 

    return img_des;
#else
    HFNetSrv hfnet_srv;
    hfnet_srv.request.image = msg;
    if (superpoint_mode) {
        if (superpoint_client.call(hfnet_srv))
        {
            auto &local_kpts = hfnet_srv.response.keypoints;
            auto &local_descriptors = hfnet_srv.response.local_descriptors;
            if (local_kpts.size() > 0)
            {
                // ROS_INFO("Received response from server desc.size %ld", desc.size());
                img_des.image_desc_size = 0;
                img_des.image_desc.clear();
                ROSPoints2LCM(local_kpts, img_des.landmarks_2d);
                img_des.landmark_num = local_kpts.size();
                img_des.feature_descriptor = local_descriptors;
                img_des.feature_descriptor_size = local_descriptors.size();
                img_des.landmarks_flag.resize(img_des.landmark_num);
                std::fill(img_des.landmarks_flag.begin(),img_des.landmarks_flag.begin()+img_des.landmark_num,0);  
                img_des.image_size = 0;
                return img_des;
            }
        }
    } else {
        if (hfnet_client.call(hfnet_srv))
        {
            auto &desc = hfnet_srv.response.global_desc;
            auto &local_kpts = hfnet_srv.response.keypoints;
            auto &local_descriptors = hfnet_srv.response.local_descriptors;
            if (desc.size() > 0)
            {
                // ROS_INFO("Received response from server desc.size %ld", desc.size());
                img_des.image_desc_size = desc.size();
                img_des.image_desc = desc;
                img_des.image_size = 0;
                ROSPoints2LCM(local_kpts, img_des.landmarks_2d);
                img_des.landmark_num = local_kpts.size();
                img_des.feature_descriptor = local_descriptors;
                img_des.feature_descriptor_size = local_descriptors.size();
                img_des.landmarks_flag.resize(img_des.landmark_num);
                std::fill(img_des.landmarks_flag.begin(),img_des.landmarks_flag.begin()+img_des.landmark_num,0);  
                return img_des;
            }
        }
    }
#endif
    ROS_INFO("FAILED on deepnet!!! Service error");
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
