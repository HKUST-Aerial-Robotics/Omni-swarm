#include <ros/ros.h>
#include <swarm_detection/drone_pose_estimator.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <time.h>


typedef std::vector<aruco::Marker> marker_array;


Pose from_cv_matrix(cv::Mat mat) {
    Eigen::Matrix4d T;
    Eigen::Matrix3d rot;
    Pose pose;
    cv::cv2eigen(mat, T);
    pose.position = T.block<3, 1>(0, 3);
    pose.attitude = T.block<3, 3>(0, 0);

    return pose;
}

int main(int argc, char* argv[]) {
    ROS_INFO("Start test detection");
     srand(time(NULL));


    aruco::MarkerDetector MDetector;
    MDetector.setDictionary("ARUCO_MIP_36h12");
    cv::Mat img_left = cv::imread("/home/xuhao/swarm_ws/src/swarm_pkgs/swarm_detection/data/mynt-left-high.png");
    cv::Mat img_right = cv::imread("/home/xuhao/swarm_ws/src/swarm_pkgs/swarm_detection/data/mynt-right-high.png");

    std::string left_cam_defs = "/home/xuhao/mf2_home/SwarmConfig/mini_mynteye_stereo/left.yaml";
    std::string right_cam_defs = "/home/xuhao/mf2_home/SwarmConfig/mini_mynteye_stereo/right.yaml";

    corner_array CorALeft;
    corner_array CorARight;
    cv::FileStorage fs_yaml("/home/xuhao/mf2_home/SwarmConfig/mini_mynteye_stereo/mini_mynteye_stereo_imu.yaml",
                            cv::FileStorage::READ);

    cv::Mat left_cam_pose;
    cv::Mat right_cam_pose;

    fs_yaml["body_T_cam0"] >> left_cam_pose;
    fs_yaml["body_T_cam1"] >> right_cam_pose;


        
    marker_array ma_left = MDetector.detect(img_left);
    marker_array ma_right = MDetector.detect(img_right);

    Camera cam_left(left_cam_defs, from_cv_matrix(left_cam_pose));
    Camera cam_right(right_cam_defs, from_cv_matrix(right_cam_pose));

    ROS_INFO("Left cam position %f %f %f", cam_left.pos().x(), cam_left.pos().y(), cam_left.pos().z());
    DroneMarker marker0(0, 0, 0.0886);
    marker0.pose.position = Eigen::Vector3d(0.1, 0, 0);



    
    
    for(auto m: ma_left){
        for (int i = 0 ;i < 4; i++){
            MarkerCornerObservsed mco(i, &marker0);
            mco.observed_point.x() = (m[i].x)/2.0;// /2 because the downsample of our camera model
            mco.observed_point.y() = (m[i].y)/2.0;
            mco.p_undist = cam_left.undist_point(mco.observed_point);

            std::cout << mco.observed_point << std::endl;
            CorALeft.push_back(mco);
        }
        
        std::cout<<m<<std::endl;    
        m.draw(img_left);
    }

    for(auto m: ma_right){
        for (int i = 0 ;i < 4; i++){
            MarkerCornerObservsed mco(i, &marker0);
            mco.observed_point.x() = (m[i].x)/2.0;// /2 because the downsample of our camera model
            mco.observed_point.y() = (m[i].y)/2.0;
            mco.p_undist = cam_right.undist_point(mco.observed_point);

            CorARight.push_back(mco);
        }
        
        std::cout<<m<<std::endl;    
//        m.draw(img_left);
    }

    camera_array ca;


    std::vector<corner_array> p_by_cam;
    ca.push_back(&cam_left);
    p_by_cam.push_back(CorALeft);

     ca.push_back(&cam_right);
     p_by_cam.push_back(CorARight);
    
    SwarmDroneDefs _sdef;
    DronePoseEstimator estimator(_sdef, ca);
    estimator.mat_to_draw_1 = img_left;
    estimator.mat_to_draw_2 = img_right;
//    estimator.mat_to_draw = img_right;


    estimator.estimate_drone_pose(p_by_cam);

//    cv::imshow("left", img_left);
//    cv::imshow("right", img_right);
//    cv::waitKey(-1);
}