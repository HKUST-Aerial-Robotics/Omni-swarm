#pragma once
#include <iostream>
#include <swarm_msgs/swarm_types.hpp>

struct SwarmLocalOutlierRejectionParams {

};

class SwarmLocalOutlierRejection {
    SwarmLocalOutlierRejectionParams param;
    
public:
    SwarmLocalOutlierRejection(SwarmLocalOutlierRejectionParams _param):
        param(_param) {
    }

    void InterLoopOutlierRejection();
};