#include "ImageDescriptor_t.hpp"
#include <swarm_msgs/ImageDescriptor.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>

inline swarm_msgs::ImageDescriptor toROSMsg(const ImageDescriptor_t & _img) {
    swarm_msgs::ImageDescriptor img_des;
    return img_des;
}


inline Pose_t fromROSPose(const geometry_msgs::Pose & pose) {
    Pose_t t;
    t.orientation[0] = pose.orientation.x;
    t.orientation[1] = pose.orientation.y;
    t.orientation[2] = pose.orientation.z;
    t.orientation[3] = pose.orientation.w;

    t.position[0] = pose.position.x;
    t.position[1] = pose.position.y;
    t.position[2] = pose.position.z;
    return t;
}


inline void ROSPoints2LCM( const std::vector<geometry_msgs::Point32> & src, std::vector<Point2d_t> & dst) {

}

inline void ROSPoints2LCM(const std::vector<geometry_msgs::Point32> & src, std::vector<Point3d_t> & dst) {
    
}