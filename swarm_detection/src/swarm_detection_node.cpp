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

using namespace swarm_msgs;
namespace enc = sensor_msgs::image_encodings;

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
class ARMarkerDetectorNode {
    ros::NodeHandle & nh;
    aruco::MarkerDetector MDetector;
    aruco::CameraParameters camera_left;
    aruco::CameraParameters camera_right;

    ros::Subscriber left_image_sub;
    ros::Subscriber right_image_sub;
    ros::Publisher armarker_pub;
    bool is_show;
    double duration;

    ros::Time last_lcam_ts;
    ros::Time last_rcam_ts;
public:
    ARMarkerDetectorNode(ros::NodeHandle & _nh):
        nh(_nh)
    {
        MDetector.setDictionary("ARUCO_MIP_36h12");
        // local_odometry_sub = nh.subscribe(vins_topic, 1, &SwarmDroneProxy::on_local_odometry_recv, this);
        left_image_sub = nh.subscribe("left_camera", 1, &ARMarkerDetectorNode::image_cb_left, this, ros::TransportHints().tcpNoDelay());
        right_image_sub = nh.subscribe("right_camera", 1, &ARMarkerDetectorNode::image_cb_right, this, ros::TransportHints().tcpNoDelay());
        armarker_pub = nh.advertise<armarker_detected >("armarker_detected", 1);

        nh.param("is_show", is_show, false);
        nh.param("duration", duration, 1.0);

        last_lcam_ts = ros::Time::now() - ros::Duration(1000000);
        last_rcam_ts = ros::Time::now() - ros::Duration(1000000);
    }

    void read_camera_params(std::string left_camera, std::string right_camera) {
        
    }

    void run_swarm_pose_estimatior(marker_array left_cam_marker, marker_array right_cam_marker) {
        //Divide marker array to drones

        //Send to drone pose estimator
    }

    void detect_cv_image(const cv::Mat & _img, int camera_id, ros::Time stamp) {

        armarker_detected ad;
        ad.header.stamp = stamp;
        ad.self_drone_id = -1;// -1 means this drone
        ad.camera_id = camera_id;

        int src_rows = _img.rows;
        int src_cols = _img.cols;
        cv::Mat img = _img;
        marker_array ma = MDetector.detect(img);
        for(auto m: ma){
            std::cout<<m<<std::endl;  
            if (is_show) {
                m.draw(img);
            }
            armarker_corner ac;
            for (int i = 0; i < 4; i++) {
                ac.marker_id = m.id;
                ac.corner_id = i;
                ac.x = m[i].x;
                ac.y = m[i].y;
                ad.corner_detected.push_back(ac);
            }
            
        }

        armarker_pub.publish(ad);
        if (is_show) {
            cv::resize(img, img, cv::Size(640, 512));
            if (camera_id == 0)
                cv::imshow("left", img);
            if (camera_id == 1)
                cv::imshow("right", img);
            cv::waitKey(10);
        }


    }

    void image_cb_left(const sensor_msgs::ImageConstPtr& msg) {
        if ((msg->header.stamp - last_lcam_ts).toSec() > duration) {
            image_cb(msg, 0);
            last_lcam_ts = msg->header.stamp;
        }

    }

    void image_cb_right(const sensor_msgs::ImageConstPtr& msg) {
        if ((msg->header.stamp - last_rcam_ts).toSec() > duration) {
            image_cb(msg, 1);
            last_rcam_ts = msg->header.stamp;
        }

    }

    void image_cb(const sensor_msgs::ImageConstPtr& msg, int camera_id)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);

            this->detect_cv_image(cv_ptr->image, camera_id, msg->header.stamp);
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
    
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
}
