#include <ros/ros.h>
#include <swarm_detection/drone_pose_estimator.h>
#include <opencv2/opencv.hpp>

typedef std::vector<aruco::Marker> marker_array;

int main(int argc, char* argv[]) {
    ROS_INFO("Start test detection");
    aruco::MarkerDetector MDetector;
    MDetector.setDictionary("ARUCO_MIP_36h12");
    cv::Mat img_left = cv::imread("/home/xuhao/swarm_ws/src/swarm_pkgs/swarm_detection/sample/image_left.png");
    cv::Mat img_right =cv::imread("/home/xuhao/swarm_ws/src/swarm_pkgs/swarm_detection/sample/image_right.png");

    std::string left_cam_defs = "/home/xuhao/mf2_home/SwarmConfig/dji_stereo/left.yaml";
    std::string right_cam_defs = "/home/xuhao/mf2_home/SwarmConfig/dji_stereo/left.yaml";
        
    Eigen::Quaterniond left_att(1, 0, 0, 0);
    Eigen::Vector3d left_pos(0, 0, 0);

    Eigen::Quaterniond right_att(1, 0, 0, 0);
    Eigen::Vector3d right_pos(0, 0.135, 0);

    marker_array ma_left = MDetector.detect(img_left);
    marker_array ma_right = MDetector.detect(img_right);

    Camera cam_left(left_cam_defs, left_att, left_pos);
    Camera cam_right(right_cam_defs, right_att, right_pos);

    DroneMarker marker0(0, 0, 0.0888);

    corner_array CorALeft;
    corner_array CorARight;

    for(auto m: ma_left){
        for (int i = 0 ;i < 4; i++){
            MarkerCornerObservsed mco(i, &marker0);
            mco.observed_point.x() = (m[i].x - 640 )/2.0;// /2 because the downsample of our camera model
            mco.observed_point.y() = (m[i].y - 512 )/2.0;
            CorALeft.push_back(mco);
        }
        
        std::cout<<m<<std::endl;    
//        m.draw(img_left);
    }

    for(auto m: ma_right){
        for (int i = 0 ;i < 4; i++){
            MarkerCornerObservsed mco(i, &marker0);
            mco.observed_point.x() = (m[i].x - 640)/2.0;// /2 because the downsample of our camera model
            mco.observed_point.y() = (m[i].y - 512)/2.0;
            CorARight.push_back(mco);
        }
        
        std::cout<<m<<std::endl;    
        m.draw(img_right);
    }

    camera_array ca;


    std::vector<corner_array> p_by_cam;
    ca.push_back(&cam_left);
    p_by_cam.push_back(CorALeft);

     ca.push_back(&cam_right);
     p_by_cam.push_back(CorARight);
    
    SwarmDroneDefs _sdef;
    DronePoseEstimator estimator(_sdef, ca);
    estimator.mat_to_draw = img_left;
//    estimator.mat_to_draw = img_right;


    estimator.estimate_drone_pose(p_by_cam);

//    cv::imshow("left", img_left);
//    cv::imshow("right", img_right);
//    cv::waitKey(-1);
}