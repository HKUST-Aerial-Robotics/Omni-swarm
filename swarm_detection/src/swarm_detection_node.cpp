#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <swarm_detection/drone_pose_estimator.h>
// #include <swarm_msgs/armarker_detected.h>
// #include <swarm_msgs/armarker_corner.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>
#include <swarm_msgs/node_detected.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <functional>
#include <stdlib.h>
#include <swarm_msgs/swarm_detected.h>



using namespace swarm_msgs;
namespace enc = sensor_msgs::image_encodings;

typedef std::vector<cv::Point2f> CVMarkerCorners;
typedef std::vector<CVMarkerCorners> marker_array;

Pose from_cv_matrix(cv::Mat mat) {
    Eigen::Matrix4d T;
    Eigen::Matrix3d rot;
    Pose pose;
    cv::cv2eigen(mat, T);
    pose.set_pos(T.block<3, 1>(0, 3));
    pose.set_att(Eigen::Quaterniond(T.block<3, 3>(0, 0)));

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
    bool undist_camera = false;
    Eigen::Vector3d ArmarkerPos = Eigen::Vector3d(-0.185, 0, 0);
    double Armarkersize = 0.92;
public:
    std::function<void(ros::Time stamp,int,Pose)> callback;
    StereoDronePoseEstimator(
            const std::string &_left_cam_def,
            const std::string &_right_cam_def,
            const std::string &_vo_config,
            std::function<void(ros::Time stamp, int,Pose)> _callback,
            Eigen::Vector3d armarker_pos,
            double armarkersize,
            bool _is_show=true, 
            bool _use_stereo = false,
            bool _undist_camera=false
            ) : callback(_callback) {

        std::cout << "Read config from " << _left_cam_def << "\n" << _right_cam_def << "\n" << _vo_config << std::endl;

        cv::FileStorage fs_yaml(_vo_config, cv::FileStorage::READ);
        cv::Mat left_cam_pose;
        cv::Mat right_cam_pose;
        fs_yaml["body_T_cam0"] >> left_cam_pose;
        fs_yaml["body_T_cam1"] >> right_cam_pose;
        use_stereo = _use_stereo;
        undist_camera = _undist_camera;
        is_show = _is_show;
        this->ArmarkerPos = armarker_pos;
        this->Armarkersize = armarkersize;

        cam_left = new Camera(_left_cam_def, from_cv_matrix(left_cam_pose));
        if (use_stereo)
            cam_right = new Camera(_right_cam_def, from_cv_matrix(right_cam_pose));


    }

    void ProcessMarkers(marker_array ma_left, marker_array ma_right) {
    }

    Pose ProcessMarkerOfNode(ros::Time stamp, int _id, CVMarkerCorners marker_left, CVMarkerCorners marker_right, cv::Mat & limg, cv::Mat & rimg) {
        DroneMarker marker0(_id, _id, Armarkersize);
        marker0.pose.set_pos(ArmarkerPos);
        // marker0.pose
        corner_array CorALeft;
        corner_array CorARight;

        if (!marker_left.empty()) {
            auto m = marker_left;
            for (int i = 0; i < 4; i++) {
                MarkerCornerObservsed mco(i, &marker0);
                mco.observed_point.x() = (double)(marker_left[i].x);
                mco.observed_point.y() = (double)(marker_left[i].y);
                if (undist_camera) {
                    ROS_ERROR("Not imply yet");
                    exit(-1);
                    // mco.p_undist = mco.observed_point - cam_left.;
                } else {
                    mco.p_undist = cam_left->undist_point(mco.observed_point);
                }
                // std::cout << "OB: " << mco.observed_point << std::endl;
                // std::cout << "UN:" << mco.p_undist << std::endl;
                CorALeft.push_back(mco);
            }

        }

        if (use_stereo) {
            if (!marker_right.empty()) {
                for (int i = 0; i < 4; i++) {
                    MarkerCornerObservsed mco(i, &marker0);
                    mco.observed_point.x() = (marker_right[i].x);
                    mco.observed_point.y() = (marker_right[i].y);
                    if (undist_camera) {
                        ROS_ERROR("Not imply yet");
                        exit(-1);
                        // mco.p_undist = mco.observed_point;
                    } else {
                        mco.p_undist = cam_left->undist_point(mco.observed_point);
                    }
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
        _sdef.drone_id = _id;
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

        Pose ret = estimator.estimate_drone_pose(p_by_cam);
        callback(stamp, _id, ret);

        return ret;
    }
};


class ARMarkerDetectorNode {
    ros::NodeHandle & nh;

    ros::Subscriber left_image_sub;
    ros::Subscriber right_image_sub;
    ros::Publisher armarker_pub;
    ros::Publisher swarm_detected_pub;
    std::map<int, ros::Publisher> remote_relative_poses_pub;

    bool is_show = false;
    double duration = 0;

    ros::Time last_lcam_ts;
    ros::Time last_rcam_ts;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::Mat last_left;
    cv::Mat last_right;
    bool use_stereo = false;
    double armarker_x = 0;
    double armarker_y = 0;
    double armarker_z = 0;
    double armarker_size = 0.1;

    StereoDronePoseEstimator *stereodronepos_est = nullptr;
public:
    ARMarkerDetectorNode(ros::NodeHandle & _nh):
            nh(_nh) {
        // local_odometry_sub = nh.subscribe(vins_topic, 1, &SwarmDroneProxy::on_local_odometry_recv, this);

        std::string left_cam_def;
        std::string right_cam_def;
        std::string vo_def;
        bool undist_camera = false;



        nh.param("is_show", is_show, false);
        nh.param("use_stereo", use_stereo, false);
        nh.param("undist_camera", undist_camera, false);

        nh.param("armarker/x", armarker_x, -0.185);
        nh.param("armarker/y", armarker_y, 0.0);
        nh.param("armarker/z", armarker_z, 0.0);
        nh.param("armarker/size", armarker_size, 0.0925);
        
        if (use_stereo) {
            ROS_INFO("Will use stereo");
        } else {
            ROS_INFO("Use single camera only");            
        };



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
                Eigen::Vector3d(armarker_x, armarker_y, armarker_z),
                armarker_size,
                is_show,
                use_stereo,
                undist_camera);

        // armarker_pub = nh.advertise<armarker_detected>("armarker_detected", 100);
        left_image_sub = nh.subscribe("left_camera", 10, &ARMarkerDetectorNode::image_cb_left, this,
                                      ros::TransportHints().tcpNoDelay());

        if (use_stereo) {
            right_image_sub = nh.subscribe("right_camera", 10, &ARMarkerDetectorNode::image_cb_right, this,
                                           ros::TransportHints().tcpNoDelay());
        }
        swarm_detected_pub = nh.advertise<swarm_msgs::swarm_detected>("swarm_detected", 100);

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
        pcs.header.frame_id = "odometry";
        pcs.pose.pose.position.x = pose.pos().x();
        pcs.pose.pose.position.y = pose.pos().y();
        pcs.pose.pose.position.z = pose.pos().z();

        pcs.pose.pose.orientation.w = pose.att().w();
        pcs.pose.pose.orientation.x = pose.att().x();
        pcs.pose.pose.orientation.y = pose.att().y();
        pcs.pose.pose.orientation.z = pose.att().z();

        pcs.pose.covariance[0] = 0.02;
        pcs.pose.covariance[6+1] = 0.01;
        pcs.pose.covariance[2*6+2] = 0.01;

        pcs.pose.covariance[3*6+3] = 5/57.3;
        pcs.pose.covariance[4*6+4] = 5/57.3;
        pcs.pose.covariance[5*6+5] = 10/57.3;

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

        // armarker_detected ad;
        // ad.header.stamp = stamp;
        // ad.self_drone_id = -1;
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

        CVMarkerCorners marker_right;
        
        swarm_detected sd;

        if (use_stereo) {
            //Not implement yet
        } else {
            // ROS_INFO("New detected!");
            sd.header.stamp = stamp;
            //-1 means this drone
            sd.self_drone_id = -1;

            for (int i = 0; i < ids_left.size(); i++) {
                if ( 10 > ids_left[i] && ids_left[i]>=0) {
                    // ROS_INFO("Prcess marker id %d", ids_left[i]);
                    auto marker_left = mal[i];
                    Pose posei = stereodronepos_est->ProcessMarkerOfNode(stamp, ids_left[i], marker_left, marker_right, limg, rimg);

                    node_detected nd;
                    nd.header.stamp = stamp;
                    nd.relpose.pose = posei.to_ros_pose();

                    nd.relpose.covariance[0] = 0.02*0.02;
                    nd.relpose.covariance[6+1] = 0.01*0.01;   
                    nd.relpose.covariance[2*6+2] = 0.01*0.01;

                    nd.relpose.covariance[3*6+3] = 5/57.3 * 5/57.3;
                    nd.relpose.covariance[4*6+4] = 5/57.3 * 5/57.3;
                    nd.relpose.covariance[5*6+5] = 10/57.3 * 10/57.3;
                    
                    nd.self_drone_id = -1;
                    nd.remote_drone_id = ids_left[i];
                    nd.is_yaw_valid = true;
                    nd.is_2d_detect = false;

                    sd.detected_nodes.push_back(nd);
                }
            }

            if (sd.detected_nodes.size() > 0) {
                swarm_detected_pub.publish(sd);
            }

        }

        double total_compute_time = (ros::Time::now() - start).toSec();
        ROS_INFO_THROTTLE(1.0, "Total Compute Time %3.2fms detected node %ld", total_compute_time*1000, sd.detected_nodes.size());
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
                this->detect_cv_image(msg->header.stamp, last_left, last_right);
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
