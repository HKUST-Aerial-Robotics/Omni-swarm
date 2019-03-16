#include <iostream>
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <swarm_detection/drone_pose_estimator.h>
#include <swarm_msgs/armarker_detected.h>
#include <swarm_msgs/armarker_corner.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

using namespace swarm_msgs;
namespace enc = sensor_msgs::image_encodings;

Pose from_cv_matrix(cv::Mat mat) {
    Eigen::Matrix4d T;
    Eigen::Matrix3d rot;
    Pose pose;
    cv::cv2eigen(mat, T);
    pose.position = T.block<3, 1>(0, 3);
    pose.attitude = T.block<3, 3>(0, 0);

    return pose;
}

void colorToGrey(cv::Mat& image_in, cv::Mat& image_out )
{   

    uchar* p_out;
    cv::Vec3b* p_in;
    for ( int idx_row = 0; idx_row < image_in.rows; ++idx_row )
    {
        p_out = image_out.ptr< uchar >( idx_row );
        p_in  = image_in.ptr< cv::Vec3b >( idx_row );

        for ( int idx_col = 0; idx_col < image_in.cols; ++idx_col )
        {
            // Red * 0.299 + Green * 0.587 + Blue * 0.114
            int dst = p_in[idx_col][0]   //
                      + p_in[idx_col][1] //
                      + p_in[idx_col][2];

            p_out[idx_col] = dst > 255 ? 255 : dst;
        }
    }
}
typedef std::vector<aruco::Marker> marker_array;


class StereoDronePoseEstimator {
    Camera *cam_left;
    Camera *cam_right;

public:
    StereoDronePoseEstimator(const std::string &_left_cam_def,
                             const std::string &_right_cam_def,
                             const std::string &_vo_config) {

        std::cout << "Read config from " << _left_cam_def << "\n" << _right_cam_def << "\n" << _vo_config << std::endl;

        cv::FileStorage fs_yaml(_vo_config, cv::FileStorage::READ);
        cv::Mat left_cam_pose;
        cv::Mat right_cam_pose;
        fs_yaml["body_T_cam0"] >> left_cam_pose;
        fs_yaml["body_T_cam1"] >> right_cam_pose;


        cam_left = new Camera(_left_cam_def, from_cv_matrix(left_cam_pose));
        cam_right = new Camera(_right_cam_def, from_cv_matrix(right_cam_pose));
    }

    void ProcessMarkers(marker_array ma_left, marker_array ma_right) {
    }

    void ProcessMarkerOfNode(aruco::Marker *marker_left, aruco::Marker *marker_right, cv::Mat limg, cv::Mat rimg) {
        DroneMarker marker0(0, 0, 0.0886);
        marker0.pose.position = Eigen::Vector3d(0.1, 0, 0);
        corner_array CorALeft;
        corner_array CorARight;


        if (marker_left != nullptr) {
            auto m = *marker_left;
            for (int i = 0; i < 4; i++) {
                MarkerCornerObservsed mco(i, &marker0);
                mco.observed_point.x() = (m[i].x) / 2.0;// /2 because the downsample of our camera model
                mco.observed_point.y() = (m[i].y) / 2.0;
                mco.p_undist = cam_left->undist_point(mco.observed_point);
                std::cout << mco.observed_point << std::endl;
                CorALeft.push_back(mco);
            }

            std::cout << m << std::endl;
//            m.draw(img_left);
        }

        if (marker_right != nullptr) {
            auto m = *marker_right;
            for (int i = 0; i < 4; i++) {
                MarkerCornerObservsed mco(i, &marker0);
                mco.observed_point.x() = (m[i].x) / 2.0;// /2 because the downsample of our camera model
                mco.observed_point.y() = (m[i].y) / 2.0;
                mco.p_undist = cam_right->undist_point(mco.observed_point);

                CorARight.push_back(mco);
            }
            std::cout << m << std::endl;
        }

        camera_array ca;


        std::vector<corner_array> p_by_cam;
        ca.push_back(cam_left);
        p_by_cam.push_back(CorALeft);

        ca.push_back(cam_right);
        p_by_cam.push_back(CorARight);

        SwarmDroneDefs _sdef;
        DronePoseEstimator estimator(_sdef, ca);
        estimator.mat_to_draw_1 = limg;
        estimator.mat_to_draw_2 = rimg;

        estimator.estimate_drone_pose(p_by_cam);
    }
};


class ARMarkerDetectorNode {
    ros::NodeHandle & nh;
    aruco::MarkerDetector MDetector;
    aruco::CameraParameters camera_left;
    aruco::CameraParameters camera_right;

    ros::Subscriber left_image_sub;
    ros::Subscriber right_image_sub;
    ros::Publisher armarker_pub;
    bool is_show = false;
    double duration = 0;

    ros::Time last_lcam_ts;
    ros::Time last_rcam_ts;


    cv::Mat last_left;
    cv::Mat last_right;

    StereoDronePoseEstimator *stereodronepos_est = nullptr;
public:
    ARMarkerDetectorNode(ros::NodeHandle & _nh):
            nh(_nh) {
        MDetector.setDictionary("ARUCO_MIP_36h12");
        // local_odometry_sub = nh.subscribe(vins_topic, 1, &SwarmDroneProxy::on_local_odometry_recv, this);

        std::string left_cam_def;
        std::string right_cam_def;
        std::string vo_def;

        nh.param("is_show", is_show, false);
        nh.param<std::string>("left_cam_def", left_cam_def,
                              "/home/xuhao/mf2_home/SwarmConfig/mini_mynteye_stereo/left.yaml");
        nh.param<std::string>("right_cam_def", right_cam_def,
                              "/home/xuhao/mf2_home/SwarmConfig/mini_mynteye_stereo/right.yaml");
        nh.param<std::string>("vo_def", vo_def,
                              "/home/xuhao/mf2_home/SwarmConfig/mini_mynteye_stereo/mini_mynteye_stereo_imu.yaml");

        nh.param("duration", duration, 1.0);

        last_lcam_ts = ros::Time::now() - ros::Duration(1000000);
        last_rcam_ts = ros::Time::now() - ros::Duration(1000000);
        stereodronepos_est = new StereoDronePoseEstimator(
                left_cam_def,
                right_cam_def,
                vo_def);

        armarker_pub = nh.advertise<armarker_detected>("armarker_detected", 1);
        left_image_sub = nh.subscribe("left_camera", 10, &ARMarkerDetectorNode::image_cb_left, this,
                                      ros::TransportHints().tcpNoDelay());
        right_image_sub = nh.subscribe("right_camera", 10, &ARMarkerDetectorNode::image_cb_right, this,
                                       ros::TransportHints().tcpNoDelay());

    }

    void read_camera_params(std::string left_camera, std::string right_camera) {
        
    }

    void run_swarm_pose_estimatior(marker_array left_cam_marker, marker_array right_cam_marker) {
        //Divide marker array to drones

        //Send to drone pose estimator
    }

    void detect_cv_image(cv::Mat &limg, cv::Mat &rimg, ros::Time stamp) {

        int src_rows = limg.rows;
        int src_cols = limg.cols;

        armarker_detected ad;
        ad.header.stamp = stamp;
        ad.self_drone_id = -1;// -1 means this drone
        // ad.camera_id = camera_id;


        assert(src_rows == rimg.rows && "Must same rows left and right");
        assert(src_cols == rimg.cols && "Must same rows left and right");


        ROS_INFO("r %d c %d", src_rows, src_cols);
        marker_array mal = MDetector.detect(limg);
        marker_array mar = MDetector.detect(rimg);

        aruco::Marker *marker_left = nullptr;
        aruco::Marker *marker_right = nullptr;
        if (mal.size() > 0) {
            marker_left = &mal[0];
        }

        if (mar.size() > 0) {
            marker_right = &mar[0];
        }


        if (marker_left != nullptr || marker_right != nullptr) {
            stereodronepos_est->ProcessMarkerOfNode(marker_left, marker_right, limg, rimg);
        }
/*

        for(auto m: mal){
            std::cout<<m<<std::endl;  
            if (is_show) {
                m.draw(limg);
            }
        }

        for(auto m: mar){
            std::cout<<m<<std::endl;  
            if (is_show) {
                m.draw(rimg);
            }
        }


        if (is_show) {
            cv::Mat out;
            hconcat(limg, rimg, out);
            cv::resize(out, out, cv::Size(src_cols, src_rows/2));
            cv::imshow("Detection", out);
        }
  */

    }

    void image_cb_left(const sensor_msgs::ImageConstPtr& msg) {
        if ((msg->header.stamp - last_lcam_ts).toSec() > duration) {
            last_lcam_ts = msg->header.stamp;
            ROS_INFO("Left!");            
            image_cb(msg, 0);
        }

    }

    void image_cb_right(const sensor_msgs::ImageConstPtr& msg) {
        if ((msg->header.stamp - last_rcam_ts).toSec() > duration) {
            last_rcam_ts = msg->header.stamp;
            ROS_INFO("Right!");                        
            image_cb(msg, 1);
        }

    }

    void image_cb(const sensor_msgs::ImageConstPtr& msg, int camera_id)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
            if (camera_id == 0) {
                last_left = cv_ptr->image.clone();
//                cv::imshow("Left", last_left);
            } else {
                last_right = cv_ptr->image.clone();
//                cv::imshow("Right", last_right);
            }

            if (fabs((last_lcam_ts - last_rcam_ts).toSec()) < 0.01) {
                this->detect_cv_image(last_left, last_right, last_lcam_ts);
            }

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

};

int main(int argc,char **argv)
{
    ros::init(argc, argv, "swarm_detection");
    ros::NodeHandle nh("swarm_detection");
    ROS_INFO("Inited swarm detection");
    ARMarkerDetectorNode ar_node(nh);

    // ros::AsyncSpinner spinner(4);
    // spinner.start();
    ros::Duration(3.0).sleep();
    ros::spin();
    ros::waitForShutdown();
}
