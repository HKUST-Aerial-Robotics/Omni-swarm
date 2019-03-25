#include <iostream>
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
#include <swarm_msgs/node_detected.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <functional>
#include <stdlib.h>

using namespace swarm_msgs;
namespace enc = sensor_msgs::image_encodings;

typedef std::vector<cv::Point2f> CVMarkerCorners;
typedef std::vector<CVMarkerCorners> marker_array;

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


class StereoDronePoseEstimator {
    Camera *cam_left;
    Camera *cam_right;
    bool is_show = true;
    bool use_stereo = true;
public:
    std::function<void(ros::Time stamp,int,Pose)> callback;
    StereoDronePoseEstimator(
            const std::string &_left_cam_def,
            const std::string &_right_cam_def,
            const std::string &_vo_config,
            std::function<void(ros::Time stamp, int,Pose)> _callback,
            bool _is_show=true,
            bool _use_stereo = false) : callback(_callback) {

        std::cout << "Read config from " << _left_cam_def << "\n" << _right_cam_def << "\n" << _vo_config << std::endl;

        cv::FileStorage fs_yaml(_vo_config, cv::FileStorage::READ);
        cv::Mat left_cam_pose;
        cv::Mat right_cam_pose;
        fs_yaml["body_T_cam0"] >> left_cam_pose;
        fs_yaml["body_T_cam1"] >> right_cam_pose;
        use_stereo = _use_stereo;

        is_show = _is_show;

        cam_left = new Camera(_left_cam_def, from_cv_matrix(left_cam_pose));
        cam_right = new Camera(_right_cam_def, from_cv_matrix(right_cam_pose));
    }

    void ProcessMarkers(marker_array ma_left, marker_array ma_right) {
    }

    void ProcessMarkerOfNode(ros::Time stamp, int _id, CVMarkerCorners marker_left, CVMarkerCorners marker_right, cv::Mat & limg, cv::Mat & rimg) {
        DroneMarker marker0(0, 0, 0.1);
        marker0.pose.position = Eigen::Vector3d(0.1, 0, 0);
        corner_array CorALeft;
        corner_array CorARight;

        if (!marker_left.empty()) {
            auto m = marker_left;
            for (int i = 0; i < 4; i++) {
                MarkerCornerObservsed mco(i, &marker0);
                mco.observed_point.x() = (double)(marker_left[i].x) / 2.0;// /2 because the downsample of our camera model
                mco.observed_point.y() = (double)(marker_left[i].y) / 2.0;
                mco.p_undist = cam_left->undist_point(mco.observed_point);
                std::cout << mco.observed_point << std::endl;
                CorALeft.push_back(mco);
            }

        }

        if (use_stereo) {
            if (!marker_right.empty()) {
                for (int i = 0; i < 4; i++) {
                    MarkerCornerObservsed mco(i, &marker0);
                    mco.observed_point.x() = (marker_right[i].x) / 2.0;// /2 because the downsample of our camera model
                    mco.observed_point.y() = (marker_right[i].y) / 2.0;
                    mco.p_undist = cam_right->undist_point(mco.observed_point);

                    CorARight.push_back(mco);
                }
            }
        }

        camera_array ca;


        std::vector<corner_array> p_by_cam;
        ca.push_back(cam_left);
        p_by_cam.push_back(CorALeft);

        if (use_stereo) {
            ca.push_back(cam_right);
            p_by_cam.push_back(CorARight);
        }

        SwarmDroneDefs _sdef;
        DronePoseEstimator estimator(_sdef, ca);
        estimator.use_ba = use_stereo;
        if (is_show) {
            estimator.mat_to_draw_1 = limg;
            if (use_stereo) {
                estimator.mat_to_draw_2 = rimg;
            }

            estimator.enable_drawing = true;
        } else {
            estimator.enable_drawing = false;
        }
        Pose pose = estimator.estimate_drone_pose(p_by_cam);

        callback(stamp, 0, pose);

    }
};


class ARMarkerDetectorNode {
    ros::NodeHandle & nh;

    ros::Subscriber left_image_sub;
    ros::Subscriber right_image_sub;
    ros::Publisher armarker_pub;
    ros::Publisher node_detected_pub;
    std::map<int, ros::Publisher> remote_relative_poses_pub;

    bool is_show = false;
    double duration = 0;

    ros::Time last_lcam_ts;
    ros::Time last_rcam_ts;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::Mat last_left;
    cv::Mat last_right;
    bool use_stereo = false;

    StereoDronePoseEstimator *stereodronepos_est = nullptr;
public:
    ARMarkerDetectorNode(ros::NodeHandle & _nh):
            nh(_nh) {
        // local_odometry_sub = nh.subscribe(vins_topic, 1, &SwarmDroneProxy::on_local_odometry_recv, this);

        std::string left_cam_def;
        std::string right_cam_def;
        std::string vo_def;

        nh.param("is_show", is_show, false);
        nh.param("use_stereo", use_stereo, false);
        
        if (use_stereo) {
            ROS_INFO("Will use stereo");
        } else {
            ROS_INFO("Use single camera only");            
        }

        nh.param<std::string>("left_cam_def", left_cam_def,
                              "/home/xuhao/mf2_home/SwarmConfig/mini_mynteye_stereo/left.yaml");
        nh.param<std::string>("right_cam_def", right_cam_def,
                              "/home/xuhao/mf2_home/SwarmConfig/mini_mynteye_stereo/right.yaml");
        nh.param<std::string>("vo_def", vo_def,
                              "/home/xuhao/mf2_home/SwarmConfig/mini_mynteye_stereo/mini_mynteye_stereo_imu.yaml");

        nh.param("duration", duration, 1.0);

        last_lcam_ts = ros::Time(0);
        last_rcam_ts = ros::Time(0);
        stereodronepos_est = new StereoDronePoseEstimator(
                left_cam_def,
                right_cam_def,
                vo_def,
                [&](ros::Time stamp, int _id, Pose pose){
                    this->on_node_detected(stamp, _id, pose);
                },
                is_show,
                use_stereo);

        armarker_pub = nh.advertise<armarker_detected>("armarker_detected", 100);
        left_image_sub = nh.subscribe("left_camera", 10, &ARMarkerDetectorNode::image_cb_left, this,
                                      ros::TransportHints().tcpNoDelay());

        if (use_stereo) {
            right_image_sub = nh.subscribe("right_camera", 10, &ARMarkerDetectorNode::image_cb_right, this,
                                           ros::TransportHints().tcpNoDelay());
        }
        node_detected_pub = nh.advertise<swarm_msgs::node_detected>("node_detected", 100);

    }

    void on_node_detected(ros::Time stamp, int _id, Pose pose) {

        if (remote_relative_poses_pub.find(_id) == remote_relative_poses_pub.end()) {
            char tname[100] = {0};
            sprintf(tname, "relative_pose_%03d", _id);
            ROS_INFO("Detected new node %d, publish to %s", _id, tname);
            remote_relative_poses_pub[_id] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(tname, 100);
        }

        ros::Publisher & _pub = remote_relative_poses_pub[_id];
        geometry_msgs::PoseWithCovarianceStamped pcs;
        pcs.header.stamp = stamp;

        //Todo, should be self frame
        pcs.header.frame_id = "world";
        pcs.pose.pose.position.x = pose.position.x();
        pcs.pose.pose.position.y = pose.position.y();
        pcs.pose.pose.position.z = pose.position.z();

        pcs.pose.pose.orientation.w = pose.attitude.w();
        pcs.pose.pose.orientation.x = pose.attitude.x();
        pcs.pose.pose.orientation.y = pose.attitude.y();
        pcs.pose.pose.orientation.z = pose.attitude.z();

        _pub.publish(pcs);

    }

    void run_swarm_pose_estimatior(marker_array left_cam_marker, marker_array right_cam_marker) {
        //Divide marker array to drones

        //Send to drone pose estimator
    }

    void detect_cv_image(ros::Time stamp, cv::Mat &limg, cv::Mat &rimg) {
        ros::Time start = ros::Time::now();
        int src_rows = limg.rows;
        int src_cols = limg.cols;

        armarker_detected ad;
        ad.header.stamp = stamp;
        ad.self_drone_id = -1;// -1 means this drone
        // ad.camera_id = camera_id;

        if (use_stereo) {
            assert(src_rows == rimg.rows && "Must same rows left and right");
            assert(src_cols == rimg.cols && "Must same rows left and right");
        }


//        ROS_INFO("r %d c %d", src_rows, src_cols);
        std::vector<int> ids_left, ids_right;
        marker_array mal, mar;
        cv::aruco::detectMarkers(limg, dictionary, mal, ids_left);

        if (use_stereo) {
            cv::aruco::detectMarkers(rimg, dictionary, mar, ids_right);
        }

        CVMarkerCorners marker_left;
        CVMarkerCorners marker_right;

        if (!mal.empty()) {
            marker_left = mal[0];
        }


        if (use_stereo) {
            //Deal with stereo
            marker_right = mar[0];
        }


        if (!marker_left.empty() || !marker_right.empty() ) {
            stereodronepos_est->ProcessMarkerOfNode(stamp, ids_left[0], marker_left, marker_right, limg, rimg);
        }

        double total_compute_time = (ros::Time::now() - start).toSec();
        ROS_INFO("Total Compute Time %3.2fms", total_compute_time*1000);
    }

    void image_cb_left(const sensor_msgs::ImageConstPtr& msg) {
        if ((msg->header.stamp - last_lcam_ts).toSec() > duration) {
            last_lcam_ts = msg->header.stamp;
        //    ROS_INFO("Left!");
            image_cb(msg, 0);
        }

    }

    void image_cb_right(const sensor_msgs::ImageConstPtr& msg) {
        if ((msg->header.stamp - last_rcam_ts).toSec() > duration) {
            last_rcam_ts = msg->header.stamp;
//            ROS_INFO("Right!");
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

            if (use_stereo) {
                if (fabs((last_lcam_ts - last_rcam_ts).toSec()) < 0.005) {
                    this->detect_cv_image(last_lcam_ts, last_left, last_right);
                }
            } else if (camera_id == 0) {
                this->detect_cv_image(last_lcam_ts, last_left, last_right);
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
//    ros::Duration(3.0).sleep();
    ros::spin();
    ros::waitForShutdown();
}
