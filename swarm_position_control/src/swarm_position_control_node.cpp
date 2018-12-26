#include <iosteam>
#include <ros/ros.h>




int main(int argc,char **argv)
{
    ros::init(argc, argv, "swarm_position_control");
    ros::NodeHandle nh("swarm_position_control");
    ROS_INFO("Inited swarm position control");
    
    ros::spin();
}
