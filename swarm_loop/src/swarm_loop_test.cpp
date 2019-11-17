#include <loop_cam.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
    ROS_INFO("SWARM_LOOP INIT");
    srand(time(NULL));

    ros::init(argc, argv, "swarm_loop_test");
    ros::NodeHandle nh("swarm_loop_test");

    std::string test_img_file = "/home/xuhao/image001.png";
    std::string camera_config_path = 
        "/home/xuhao/swarm_ws/src/VINS-Fusion-gpu/config/realsense/left.yaml";
    std::string BRIEF_PATTHER_FILE = "/home/xuhao/swarm_ws/src/VINS-Fusion-gpu/support_files/brief_pattern.yml";
    
    ros::Publisher img_des_pub = nh.advertise<swarm_msgs::ImageDescriptor>("/swarm_loop/new_image_des", 1);
    
    cv::Mat img = cv::imread(test_img_file.c_str(), cv::IMREAD_GRAYSCALE);
    LoopCam cam(camera_config_path, BRIEF_PATTHER_FILE);
    auto des = cam.feature_detect(img);
    ros::Rate r(10);
    while (true) {
        ROS_INFO("Publishing the descriptors....");
        img_des_pub.publish(des);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}