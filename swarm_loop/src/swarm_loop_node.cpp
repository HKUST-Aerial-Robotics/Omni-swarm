#include "swarm_loop.h"

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
    backward::SignalHandling sh;
}

class SwarmLoopNode :  public swarm_localization_pkg::SwarmLoop
{
    public:
        SwarmLoopNode(ros::NodeHandle & nh)
        {
            Init(nh);
        }
};

int main(int argc, char **argv)
{
    cv::setNumThreads(1);
    ros::init(argc, argv, "swarm_loop");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);


    SwarmLoopNode fisheye(n);
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    return 0;
}

