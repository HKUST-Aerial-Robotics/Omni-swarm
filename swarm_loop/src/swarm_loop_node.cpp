#include "ros/ros.h"
#include <iostream>

int main(int argc, char **argv) {
    ROS_INFO("SWARM_LOOP INIT");
    //Use time as seed
    srand(time(NULL));
    ros::init(argc, argv, "swarm_loop");
    ros::NodeHandle nh("swarm_loop");
    SwarmLocalizationNode uwbfusernode(nh);

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();
    //ros::spin();

    return 0;
}
